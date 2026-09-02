#include "carla_telemetry/ros2_backend.hpp"

#include <carla/client/World.h>
#include <carla/client/WorldSnapshot.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/VehicleLightState.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <nlohmann/json.hpp>

#include "carla_telemetry/perf_monitor.hpp"
#include "carla_telemetry/sensors/battery.hpp"
#include "carla_telemetry/sensors/sensor_clock.hpp"
#include "carla_telemetry/vehicle.hpp"

namespace carla_telemetry {

namespace {
std::string get_or(const std::unordered_map<std::string, std::string>& m,
                   const std::string& key, const std::string& def) {
  auto it = m.find(key);
  return it != m.end() ? it->second : def;
}

// Below this delta a physics command is treated as a repeat of the value
// already applied and skipped.
constexpr float kFrictionEpsilon = 1e-4f;

// Used only if the physics control reports no wheels; matches the configured
// vehicle.physics.wheels[].radius.
constexpr double kFallbackWheelRadius = 0.25;

// Below this chassis speed the slip ratio's divisor makes it meaningless.
constexpr double kMinSlipRatioSpeed = 0.5;

// Convert wall-clock epoch seconds to ROS Time stamp.
builtin_interfaces::msg::Time epoch_to_stamp(double t) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(t);
  stamp.nanosec = static_cast<uint32_t>((t - stamp.sec) * 1e9);
  return stamp;
}
}  // namespace

CarlaROS2Backend::CarlaROS2Backend(
    rclcpp_lifecycle::LifecycleNode* node,
    const std::unordered_map<std::string, std::string>& topics_cfg,
    const std::unordered_map<std::string, std::string>& services_cfg,
    const std::unordered_map<std::string, std::string>& qos_cfg,
    const std::string& ns, rclcpp::CallbackGroup::SharedPtr cb_group)
    : node_(node),
      namespace_(ns),
      cb_group_(std::move(cb_group)),
      topics_cfg_(topics_cfg),
      services_cfg_(services_cfg),
      qos_cfg_(qos_cfg) {
  // Applied to every subscription below so none of them lands in the node's
  // default group alongside the lifecycle services.
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  auto get_qos = [this](const std::string& key) {
    auto it = qos_cfg_.find(key);
    if (it != qos_cfg_.end() && it->second == "BEST_EFFORT") {
      return rclcpp::QoS(1).best_effort();
    }
    return rclcpp::QoS(1).reliable();
  };

  auto qos_rel = rclcpp::QoS(1).reliable();

  // Core feedback publishers
  gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      topic(get_or(topics_cfg_, "feedback_gps", "feedback/gps")),
      get_qos("gps"));
  gps_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      topic(get_or(topics_cfg_, "feedback_gps_vel", "feedback/gps_vel")),
      get_qos("gps"));
  battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
      topic(get_or(topics_cfg_, "feedback_battery_state",
                   "feedback/battery/state")),
      get_qos("battery"));
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      topic(get_or(topics_cfg_, "feedback_imu", "feedback/imu")),
      get_qos("imu"));
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      topic(get_or(topics_cfg_, "odom", "odom")), get_qos("odometry"));
  boxes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      topic(get_or(topics_cfg_, "ground_truth_boxes", "ground_truth/boxes")),
      get_qos("ground_truth_boxes"));
  speed_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
      topic(get_or(topics_cfg_, "feedback_speed", "feedback/speed")), qos_rel);
  steer_echo_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
      topic(get_or(topics_cfg_, "feedback_steering_angle",
                   "feedback/steering_angle")),
      qos_rel);
  steer_angles_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      topic(get_or(topics_cfg_, "feedback_steering_angles",
                   "feedback/steering_angles")),
      qos_rel);
  motors_pub_ = node_->create_publisher<std_msgs::msg::String>(
      topic(get_or(topics_cfg_, "feedback_motors", "feedback/motors")),
      qos_rel);
  tire_forces_pub_ = node_->create_publisher<sim_manager_msgs::msg::TireForces>(
      topic(
          get_or(topics_cfg_, "feedback_tire_forces", "feedback/tire_forces")),
      qos_rel);
  vehicle_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
      topic(get_or(topics_cfg_, "feedback_vehicle_state",
                   "feedback/vehicle_state")),
      qos_rel);
  autonomous_mode_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      topic(get_or(topics_cfg_, "feedback_autonomous_mode",
                   "feedback/autonomous_mode")),
      qos_rel);
  clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());

  // Control subscriptions (moved to set_control_config to respect control mode)
  spawn_sub_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
      "/sim/spawn_point", qos_rel,
      [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        if (!vehicle_actor_) return;
        auto tf = vehicle_actor_->GetTransform();
        tf.location.x = static_cast<float>(msg->x);
        tf.location.y = -static_cast<float>(msg->y);
        tf.rotation.yaw = -static_cast<float>(msg->theta * 180.0 / M_PI);
        vehicle_actor_->SetTransform(tf);
      },
      sub_opts);

  // Sim lifecycle services
  auto ok_resp = [](const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr resp) {
    resp->success = true;
    resp->message = "OK";
  };
  sim_start_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/sim/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        sim_running_ = true;
        if (vehicle_) vehicle_->resume();
        resp->success = true;
        resp->message = "Start.";
      },
      rmw_qos_profile_services_default, cb_group_);
  sim_stop_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/sim/stop",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        sim_running_ = false;
        if (vehicle_) vehicle_->pause();
        resp->success = true;
        resp->message = "Stop.";
      },
      rmw_qos_profile_services_default, cb_group_);
  sim_start_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/sim/start", qos_rel,
      [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        sim_running_ = true;
        if (vehicle_) vehicle_->resume();
      },
      sub_opts);
  sim_stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/sim/stop", qos_rel,
      [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        sim_running_ = false;
        if (vehicle_) vehicle_->pause();
      },
      sub_opts);

  // Runtime environment tuning: tire-to-ground friction and aerodynamic drag.
  // Both go straight to ApplyPhysicsControl, so they take effect on the next
  // physics step without respawning the vehicle.
  tire_friction_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      topic(get_or(topics_cfg_, "control_tire_friction",
                   "control/tire_friction")),
      qos_rel,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        apply_tire_friction(msg->data);
      },
      sub_opts);
  drag_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      topic(get_or(topics_cfg_, "control_drag_coefficient",
                   "control/drag_coefficient")),
      qos_rel,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        apply_drag_coefficient(msg->data);
      },
      sub_opts);

  // Battery services
  auto svc = services_cfg_;
  start_drain_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      topic(get_or(svc, "start_drain", "control/battery/start_drain")),
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr r) {
        if (battery_) battery_->start_draining();
        r->success = true;
        r->message = "Drain started.";
      },
      rmw_qos_profile_services_default, cb_group_);
  stop_drain_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      topic(get_or(svc, "stop_drain", "control/battery/stop_drain")),
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr r) {
        if (battery_) battery_->stop_draining();
        r->success = true;
        r->message = "Drain stopped.";
      },
      rmw_qos_profile_services_default, cb_group_);
  start_charge_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      topic(get_or(svc, "start_charge", "control/battery/start_charge")),
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr r) {
        if (battery_) battery_->start_charging();
        r->success = true;
        r->message = "Charge started.";
      },
      rmw_qos_profile_services_default, cb_group_);
  stop_charge_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      topic(get_or(svc, "stop_charge", "control/battery/stop_charge")),
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr r) {
        if (battery_) battery_->stop_charging();
        r->success = true;
        r->message = "Charge stopped.";
      },
      rmw_qos_profile_services_default, cb_group_);

  // Light services
  auto make_light_cb = [this](uint32_t flag) {
    return [this, flag](const std_srvs::srv::SetBool::Request::SharedPtr req,
                        std_srvs::srv::SetBool::Response::SharedPtr resp) {
      if (!vehicle_actor_) {
        resp->success = false;
        resp->message = "No vehicle.";
        return;
      }
      auto v =
          boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
      if (!v) {
        resp->success = false;
        return;
      }
      set_light_bit(*v, flag, req->data);  // updates the cached bitmask too
      resp->success = true;
      resp->message = "Toggled.";
    };
  };
  high_beams_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      topic(get_or(svc, "high_beams", "control/lights/high_beams")),
      make_light_cb(static_cast<uint32_t>(
          carla::rpc::VehicleLightState::LightState::HighBeam)),
      rmw_qos_profile_services_default, cb_group_);
  low_beams_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      topic(get_or(svc, "low_beams", "control/lights/low_beams")),
      make_light_cb(static_cast<uint32_t>(
          carla::rpc::VehicleLightState::LightState::LowBeam)),
      rmw_qos_profile_services_default, cb_group_);
  left_blinker_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      topic(get_or(svc, "left_blinker", "control/lights/left_blinker")),
      make_light_cb(static_cast<uint32_t>(
          carla::rpc::VehicleLightState::LightState::LeftBlinker)),
      rmw_qos_profile_services_default, cb_group_);
  right_blinker_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      topic(get_or(svc, "right_blinker", "control/lights/right_blinker")),
      make_light_cb(static_cast<uint32_t>(
          carla::rpc::VehicleLightState::LightState::RightBlinker)),
      rmw_qos_profile_services_default, cb_group_);
  siren_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      topic(get_or(svc, "siren", "control/lights/siren")),
      make_light_cb(static_cast<uint32_t>(
          carla::rpc::VehicleLightState::LightState::Special1)),
      rmw_qos_profile_services_default, cb_group_);
  brake_lights_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      topic(get_or(svc, "brake_lights", "control/lights/brake_lights")),
      make_light_cb(static_cast<uint32_t>(
          carla::rpc::VehicleLightState::LightState::Brake)),
      rmw_qos_profile_services_default, cb_group_);
  force_manual_control_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      topic(
          get_or(svc, "force_manual_control", "control/force_manual_control")),
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr resp) {
        if (req->data && !manual_control_override_) {
          manual_control_override_ = true;
          original_control_mode_ = control_mode_;
          control_mode_.steer_vel_pid = PIDConfig{};
          control_mode_.rpm_pid = PIDConfig{};
          resp->message = "Manual control enabled (PID zeroed)";
        } else if (!req->data && manual_control_override_) {
          manual_control_override_ = false;
          control_mode_ = original_control_mode_;
          // Reset all velocity commands and PID states on transition
          // to autonomous, so stale manual throttle cannot override brake
          cmd_velocity_rpm_ = 0.0;
          cmd_steering_deg_ = 0.0;
          ack_speed_ = 0.0;
          ack_steering_angle_ = 0.0;
          ack_steering_angle_vel_ = 0.0;
          ack_acceleration_ = 0.0;
          ack_jerk_ = 0.0;
          rpm_pid_ = PIDState{};
          steer_vel_pid_ = PIDState{};
          resp->message =
              "Manual control disabled (PID restored, commands reset)";
        } else {
          resp->message = req->data ? "Manual control already enabled"
                                    : "Manual control already disabled";
        }
        resp->success = true;
      },
      rmw_qos_profile_services_default, cb_group_);
  set_steering_mode_srv_ =
      node_->create_service<carla_msgs::srv::SetSteeringMode>(
          topic(get_or(svc, "set_steering_mode", "control/set_steering_mode")),
          [this](const carla_msgs::srv::SetSteeringMode::Request::SharedPtr req,
                 carla_msgs::srv::SetSteeringMode::Response::SharedPtr resp) {
            steering_mode_ = req->mode;
            if (vehicle_actor_) {
              auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(
                  vehicle_actor_);
              if (v) {
                auto ctrl = v->GetControl();
                ctrl.steering_mode = steering_mode_;
                v->ApplyControl(ctrl);
              }
            }
            resp->success = true;
            resp->message = "Steering mode set to " + std::to_string(req->mode);
          },
          rmw_qos_profile_services_default, cb_group_);

  RCLCPP_INFO(node_->get_logger(), "[CarlaROS2Backend] Initialized.");
}

std::string CarlaROS2Backend::topic(const std::string& suffix) const {
  if (suffix.empty()) return "/";
  if (suffix[0] == '/') return suffix;
  if (namespace_.empty()) return "/" + suffix;
  return "/" + namespace_ + "/" + suffix;
}

void CarlaROS2Backend::set_control_config(double max_rpm, double max_steer_deg,
                                          const ControlModeConfig& mode_cfg) {
  max_rpm_ = max_rpm;
  max_steer_deg_ = max_steer_deg;
  control_mode_ = mode_cfg;

  if (control_mode_.source == "ackermann_drive" && vehicle_actor_) {
    auto v =
        boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
    if (v) {
      const auto& ac = control_mode_.ackermann_controller;
      carla::rpc::AckermannControllerSettings settings;
      settings.speed_kp = static_cast<float>(ac.speed_kp);
      settings.speed_ki = static_cast<float>(ac.speed_ki);
      settings.speed_kd = static_cast<float>(ac.speed_kd);
      settings.accel_kp = static_cast<float>(ac.accel_kp);
      settings.accel_ki = static_cast<float>(ac.accel_ki);
      settings.accel_kd = static_cast<float>(ac.accel_kd);
      v->ApplyAckermannControllerSettings(settings);
    }
  }

  auto qos_rel = rclcpp::QoS(1).reliable();
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  brake_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic(get_or(topics_cfg_, "control_brake", "control/brake")), qos_rel,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        cmd_brake_ = msg->data;
        if (cmd_brake_) {
          // Brake is highest priority — zero out all velocity commands
          // so no stale throttle can override the brake
          cmd_velocity_rpm_ = 0.0;
          ack_speed_ = 0.0;
        }
      },
      sub_opts);

  if (control_mode_.source == "ackermann_drive") {
    ackermann_sub_ =
        node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            topic(get_or(topics_cfg_, "control_ackermann",
                         "control/ackermann_drive")),
            qos_rel,
            [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr
                       msg) {
              ack_speed_ = msg->drive.speed;
              ack_steering_angle_ = msg->drive.steering_angle;
              ack_steering_angle_vel_ = msg->drive.steering_angle_velocity;
              ack_acceleration_ = msg->drive.acceleration;
              ack_jerk_ = msg->drive.jerk;
            },
            sub_opts);
  } else {
    vel_rpm_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        topic(get_or(topics_cfg_, "control_velocity_rpm",
                     "control/velocity_rpm")),
        qos_rel,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          cmd_velocity_rpm_ = msg->data;
        },
        sub_opts);
    steer_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        topic(get_or(topics_cfg_, "control_steering_angle_deg",
                     "control/steering_angle_deg")),
        qos_rel,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          cmd_steering_deg_ = msg->data;
        },
        sub_opts);
  }
}

// Camera / LiDAR registration

void CarlaROS2Backend::register_camera(const std::string& name,
                                       const std::string& topic_rgb,
                                       const std::string& topic_info,
                                       const std::string& qos_reliability) {
  if (camera_pubs_.count(name)) return;
  auto qos = (qos_reliability == "RELIABLE") ? rclcpp::QoS(1).reliable()
                                             : rclcpp::QoS(1).best_effort();
  auto rgb_t = topic(topic_rgb.empty() ? name + "/rgb" : topic_rgb);
  auto info_t = topic(topic_info.empty() ? name + "/camera_info" : topic_info);
  camera_pubs_[name] = {
      node_->create_publisher<sensor_msgs::msg::Image>(rgb_t, qos),
      node_->create_publisher<sensor_msgs::msg::CameraInfo>(info_t, qos),
  };
}

void CarlaROS2Backend::register_lidar(const std::string& name,
                                      const std::string& tp,
                                      const std::string& qos_reliability) {
  if (lidar_pubs_.count(name)) return;
  auto qos = (qos_reliability == "RELIABLE") ? rclcpp::QoS(1).reliable()
                                             : rclcpp::QoS(1).best_effort();
  auto t = topic(tp.empty() ? "lidar/" + name + "/points" : tp);
  lidar_pubs_[name] =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(t, qos);
}

// Publish GPS

void CarlaROS2Backend::publish_gps(const GpsState& g) {
  auto stamp = epoch_to_stamp(g.capture_time);
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = stamp;
  fix.header.frame_id = "gps_link";
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix.latitude = g.latitude;
  fix.longitude = g.longitude;
  fix.altitude = g.altitude;
  double eph2 = g.eph * g.eph, epv2 = g.epv * g.epv;
  fix.position_covariance = {eph2, 0, 0, 0, eph2, 0, 0, 0, epv2};
  fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  gps_pub_->publish(fix);

  geometry_msgs::msg::TwistStamped tw;
  tw.header.stamp = stamp;
  tw.header.frame_id = "map";
  tw.twist.linear.x = g.velocity_east;
  tw.twist.linear.y = g.velocity_north;
  tw.twist.linear.z = -g.velocity_down;
  gps_vel_pub_->publish(tw);
}

// Publish Battery

void CarlaROS2Backend::publish_battery(const BatteryState& b) {
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = epoch_to_stamp(b.capture_time);
  msg.header.frame_id = "base_link";
  msg.voltage = b.voltage;
  msg.current = b.current;
  msg.charge = b.charge;
  msg.capacity = b.capacity;
  msg.design_capacity = b.capacity;
  msg.percentage = b.charge_fraction;
  msg.temperature = b.temperature;
  msg.power_supply_status =
      b.is_depleted
          ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING
          : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  msg.present = true;
  battery_pub_->publish(msg);
}

// Publish IMU

void CarlaROS2Backend::publish_imu(const ImuState& s) {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = epoch_to_stamp(s.capture_time);
  msg.header.frame_id = s.frame_id;
  msg.linear_acceleration.x = s.accel_x;
  msg.linear_acceleration.y = -s.accel_y;
  msg.linear_acceleration.z = s.accel_z;
  msg.angular_velocity.x = s.gyro_x;
  msg.angular_velocity.y = -s.gyro_y;
  msg.angular_velocity.z = s.gyro_z;
  msg.orientation.x = s.qx;
  msg.orientation.y = s.qy;
  msg.orientation.z = s.qz;
  msg.orientation.w = s.qw;
  imu_pub_->publish(msg);
}

// Publish Camera

void CarlaROS2Backend::publish_camera_image(CameraData& d) {
  if (camera_pubs_.find(d.frame_id) == camera_pubs_.end())
    register_camera(d.frame_id);
  auto stamp = builtin_interfaces::msg::Time();
  stamp.sec = static_cast<int32_t>(d.capture_time);
  stamp.nanosec = static_cast<uint32_t>((d.capture_time - stamp.sec) * 1e9);

  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = d.frame_id;
  msg.height = d.height;
  msg.width = d.width;
  msg.encoding = d.encoding;
  msg.is_bigendian = 0;
  msg.step = d.width * 4;       // BGRA: 4 bytes/pixel
  msg.data = std::move(d.rgb);  // move: caller's CameraData is a throwaway;
                                // skips a full-frame copy
  camera_pubs_[d.frame_id].rgb->publish(msg);
}

void CarlaROS2Backend::publish_camera_info(const CameraData& d) {
  if (camera_pubs_.find(d.frame_id) == camera_pubs_.end())
    register_camera(d.frame_id);
  auto stamp = builtin_interfaces::msg::Time();
  stamp.sec = static_cast<int32_t>(d.capture_time);
  stamp.nanosec = static_cast<uint32_t>((d.capture_time - stamp.sec) * 1e9);

  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = d.frame_id;
  msg.height = d.height;
  msg.width = d.width;
  msg.distortion_model = "plumb_bob";
  msg.d = {0, 0, 0, 0, 0};
  for (int i = 0; i < 9; ++i) msg.k[i] = d.intrinsics[i];
  msg.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  msg.p = {d.intrinsics[0],
           0,
           d.intrinsics[2],
           0,
           0,
           d.intrinsics[4],
           d.intrinsics[5],
           0,
           0,
           0,
           1,
           0};
  camera_pubs_[d.frame_id].info->publish(msg);
}

// Publish Odometry

void CarlaROS2Backend::publish_odometry(const OdometryState& o) {
  auto stamp = epoch_to_stamp(o.capture_time);
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = o.frame_id;
  msg.child_frame_id = o.child_frame_id;
  msg.pose.pose.position.x = o.pos_x;
  msg.pose.pose.position.y = o.pos_y;
  msg.pose.pose.position.z = o.pos_z;
  msg.pose.pose.orientation.x = o.qx;
  msg.pose.pose.orientation.y = o.qy;
  msg.pose.pose.orientation.z = o.qz;
  msg.pose.pose.orientation.w = o.qw;
  for (int i = 0; i < 36; ++i) msg.pose.covariance[i] = 0.0;
  for (int i = 0; i < 6; ++i) msg.pose.covariance[i * 7] = 0.001;
  msg.twist.twist.linear.x = o.vx;
  msg.twist.twist.linear.y = o.vy;
  msg.twist.twist.linear.z = o.vz;
  msg.twist.twist.angular.x = o.wx;
  msg.twist.twist.angular.y = o.wy;
  msg.twist.twist.angular.z = o.wz;
  for (int i = 0; i < 36; ++i) msg.twist.covariance[i] = 0.0;
  for (int i = 0; i < 6; ++i) msg.twist.covariance[i * 7] = 0.001;
  odom_pub_->publish(msg);

  if (o.broadcast_tf) {
    if (!tf_broadcaster_)
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = o.frame_id;
    t.child_frame_id = o.child_frame_id;
    t.transform.translation.x = o.pos_x;
    t.transform.translation.y = o.pos_y;
    t.transform.translation.z = o.pos_z;
    t.transform.rotation.x = o.qx;
    t.transform.rotation.y = o.qy;
    t.transform.rotation.z = o.qz;
    t.transform.rotation.w = o.qw;
    tf_broadcaster_->sendTransform(t);
  }
}

// Publish ground-truth 3D bounding boxes

void CarlaROS2Backend::publish_ground_truth_boxes(const GroundTruthBoxes& gt) {
  auto stamp = epoch_to_stamp(gt.capture_time);
  visualization_msgs::msg::MarkerArray arr;

  // Stale markers from departed objects auto-expire via lifetime below.
  builtin_interfaces::msg::Duration lifetime;
  lifetime.sec = 0;
  lifetime.nanosec = 200000000;  // 0.2 s

  for (const auto& b : gt.boxes) {
    // Skip ghost boxes: empty label or degenerate (zero) scale.
    if (b.label.empty() || b.sx < 1e-3 || b.sy < 1e-3 || b.sz < 1e-3) {
      continue;
    }

    // Label string, reused for cube.text (RViz ignores it) so no marker
    // serializes an empty text field.
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s|%u|%d|%.2f|%.2f|%s", b.label.c_str(),
                  b.track_id, static_cast<int>(b.score), b.vx, b.vy,
                  b.attribute.c_str());

    // Cube
    visualization_msgs::msg::Marker cube;
    cube.header.frame_id = gt.frame_id;
    cube.header.stamp = stamp;
    cube.ns = "gt_boxes";

    cube.id = static_cast<int>(b.track_id);
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.pose.position.x = b.px;
    cube.pose.position.y = b.py;
    cube.pose.position.z = b.pz;
    cube.pose.orientation.x = b.qx;
    cube.pose.orientation.y = b.qy;
    cube.pose.orientation.z = b.qz;
    cube.pose.orientation.w = b.qw;
    cube.scale.x = b.sx;
    cube.scale.y = b.sy;
    cube.scale.z = b.sz;
    cube.color.r = b.is_ego ? 0.0f : 1.0f;
    cube.color.g = b.is_ego ? 1.0f : 0.0f;
    cube.color.b = 0.0f;
    cube.color.a = 0.5f;
    cube.text = buf;
    cube.lifetime = lifetime;
    arr.markers.push_back(cube);
  }

  boxes_pub_->publish(arr);
}

// Publish PointCloud2

void CarlaROS2Backend::publish_point_cloud(const LidarData& d) {
  if (d.data.empty() || d.num_points == 0) return;
  std::string key = d.lidar_name.empty() ? d.frame_id : d.lidar_name;
  if (!lidar_pubs_.count(key)) register_lidar(key, "lidar/" + key + "/points");

  const uint32_t n = static_cast<uint32_t>(d.num_points);
  constexpr int point_step = 16;  // interleaved XYZI float32

  sensor_msgs::msg::PointCloud2 msg;
  auto stamp = builtin_interfaces::msg::Time();
  stamp.sec = static_cast<int32_t>(d.capture_time);
  stamp.nanosec = static_cast<uint32_t>((d.capture_time - stamp.sec) * 1e9);
  msg.header.stamp = stamp;
  msg.header.frame_id = d.frame_id;
  msg.height = 1;
  msg.width = n;
  msg.fields.resize(4);
  auto mkf = [](const std::string& nm, uint32_t off) {
    sensor_msgs::msg::PointField f;
    f.name = nm;
    f.offset = off;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    return f;
  };
  msg.fields[0] = mkf("x", 0);
  msg.fields[1] = mkf("y", 4);
  msg.fields[2] = mkf("z", 8);
  msg.fields[3] = mkf("intensity", 12);
  msg.is_bigendian = false;
  msg.point_step = point_step;
  msg.row_step = point_step * n;
  msg.is_dense = true;

  // Data already interleaved XYZI — single bulk copy into the message buffer.
  msg.data = d.data;
  lidar_pubs_[key]->publish(msg);
}

// Speed / Steering

void CarlaROS2Backend::publish_speed(double speed_ms) {
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(speed_ms);
  speed_pub_->publish(msg);
}

void CarlaROS2Backend::publish_steering_angles(double steer_deg) {
  float steer_FL = 0.0f;
  float steer_FR = 0.0f;
  float steer_RL = 0.0f;
  float steer_RR = 0.0f;

  if (std::isnan(steer_deg)) {
    if (vehicle_actor_) {
      auto v =
          boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
      if (v) {
        steer_FL =
            -v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::FL_Wheel);
        steer_FR =
            -v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::FR_Wheel);
        steer_RL =
            -v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::BL_Wheel);
        steer_RR =
            -v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::BR_Wheel);
      } else {
        steer_FL = steer_FR = static_cast<float>(last_steering_deg_);
      }
    } else {
      steer_FL = steer_FR = static_cast<float>(last_steering_deg_);
    }
    steer_deg = (steer_FL + steer_FR) / 2.0;
  } else {
    steer_FL = steer_FR = static_cast<float>(steer_deg);
  }

  std_msgs::msg::Float32 echo;
  echo.data = static_cast<float>(steer_deg);
  steer_echo_pub_->publish(echo);

  sensor_msgs::msg::JointState js;
  // Sim-clock domain, same as every sensor stamp and /clock — NOT the node's
  // system clock. The server runs at a different pace than wall time (sync
  // mode ticks as fast as it can), so the two domains drift apart without
  // bound; mixing them makes this topic un-fusable with /odom, the LiDAR and
  // the cameras.
  js.header.stamp = epoch_to_stamp(sim_now_epoch());
  js.name = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};
  js.position = {static_cast<double>(steer_FL), static_cast<double>(steer_FR),
                 static_cast<double>(steer_RL), static_cast<double>(steer_RR)};
  js.velocity = {0, 0, 0, 0};
  js.effort = {0, 0, 0, 0};
  steer_angles_pub_->publish(js);
}

void CarlaROS2Backend::publish_motors() {
  if (!vehicle_actor_) return;
  auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
  if (!v) return;

  auto telem = v->GetTelemetryData();
  auto phys = v->GetPhysicsControl();

  // Wheel radius
  float radius_FL = 0.36f, radius_FR = 0.36f, radius_RL = 0.36f,
        radius_RR = 0.36f;
  if (phys.wheels.size() >= 4) {
    radius_FL = phys.wheels[0].radius / 100.0f;
    radius_FR = phys.wheels[1].radius / 100.0f;
    radius_RL = phys.wheels[2].radius / 100.0f;
    radius_RR = phys.wheels[3].radius / 100.0f;
  }

  float steer_FL =
      v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::FL_Wheel);
  float steer_FR =
      v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::FR_Wheel);
  float steer_RL =
      v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::BL_Wheel);
  float steer_RR =
      v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::BR_Wheel);

  double timestamp_ms = sim_now_epoch() * 1000.0;

  nlohmann::json j = {{"timestamp", timestamp_ms}, {"frame_id", "Motors_Data"}};

  if (telem.wheels.size() >= 4) {
    auto from_wheel_state = [](const carla::rpc::WheelTelemetryData& w,
                               float radius_m, float steer_deg, float brake) {
      float speed_mps = w.omega * radius_m;
      float speed_rpm = (w.omega * 60.0f) / (2.0f * M_PI);
      float power_W = w.torque * std::abs(w.omega);
      return nlohmann::json{{"speed_rpm", speed_rpm},
                            {"speed_mps", speed_mps},
                            {"steering_angle_deg", steer_deg},
                            {"steering_angle_rad", steer_deg * M_PI / 180.0f},
                            {"steering_angular_velocity", 0.0f},
                            {"steering_angular_velocity_radps", 0.0f},
                            {"brake_percentage", brake * 100.0f},
                            {"power", power_W},
                            {"torque", w.torque},
                            {"drive_motor_error", "OK"},
                            {"steering_motor_error", "OK"},
                            {"brake_motor_error", "OK"}};
    };

    // telem.brake, not GetControl().brake — the latter reads a flat 0 while
    // CARLA's native Ackermann controller is driving the vehicle.
    j["front_left"] =
        from_wheel_state(telem.wheels[0], radius_FL, steer_FL, telem.brake);
    j["front_right"] =
        from_wheel_state(telem.wheels[1], radius_FR, steer_FR, telem.brake);
    j["back_left"] =
        from_wheel_state(telem.wheels[2], radius_RL, steer_RL, telem.brake);
    j["back_right"] =
        from_wheel_state(telem.wheels[3], radius_RR, steer_RR, telem.brake);
  } else {
    return;  // No wheels to publish
  }

  std_msgs::msg::String msg;
  msg.data = j.dump();
  motors_pub_->publish(msg);
}

void CarlaROS2Backend::publish_tire_forces(
    const carla::rpc::VehicleTelemetryData& telem, double capture_epoch,
    uint64_t capture_frame) {
  if (telem.wheels.size() < 4) return;
  if (capture_frame != 0 && capture_frame == last_tire_frame_) return;
  last_tire_frame_ = capture_frame;

  // CARLA's own per-wheel telemetry, passed through except for the slip ratio
  // (a kinematic identity in omega and radius, see below). There used to be
  // an analytic tire model here — a Magic Formula for lateral force and a
  // Liniger-style Frx for longitudinal — and both were measurably wrong
  // against the vehicle they described (2026-08-14, live): the Magic Formula
  // published 7.0-7.5x the lateral force the chassis actually felt (fitted
  // B*C = 2.69 vs the configured 19.0 over two clean cornering runs, R2 =
  // 0.89), and the drive-force constant implied 133 m/s2 at full throttle on a
  // 240 kg car, 4.6x past its own mu*m*g traction limit. CARLA computes these
  // forces itself as part of the physics step, so reporting its numbers makes
  // the topic incapable of disagreeing with the simulation and removes every
  // constant that had to be re-identified whenever the vehicle changed.
  sim_manager_msgs::msg::TireForces msg;
  // The world frame the telemetry RPC was served on, converted through the
  // shared sim clock: byte-identical to the /odom, IMU and LiDAR stamps of
  // that frame. A wall-extrapolated sim_now_epoch() reading was measured
  // 85-111 ms (up to 3 server frames) away from the frame it described, since
  // the telemetry thread is paced in wall time and the extrapolation cannot
  // know which frame the server answered from.
  msg.stamp = epoch_to_stamp(capture_epoch);
  msg.wheel_names = {"FL", "FR", "RL", "RR"};

  // Wheel radii for the slip ratio below, in metres. Served from the physics
  // cache, so no RPC; the config value is centimetres.
  std::array<double, 4> radius_m{kFallbackWheelRadius, kFallbackWheelRadius,
                                 kFallbackWheelRadius, kFallbackWheelRadius};
  if (auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(
          vehicle_actor_)) {
    const auto& phys = physics(*v);
    for (size_t i = 0; i < 4 && i < phys.wheels.size(); ++i) {
      radius_m[i] = phys.wheels[i].radius / 100.0;
    }
  }
  const double chassis_speed = telem.speed;

  for (size_t i = 0; i < 4; ++i) {
    const auto& w = telem.wheels[i];
    // CARLA's WheelTelemetryData.lat_slip is in DEGREES, and its sign is the
    // left-handed one, so negating it to reach the ROS body frame and then
    // converting is the same as converting the raw value: alpha_ros =
    // -alpha_carla = +lat_slip. Verified live in a steady 4 m/s / 0.20 rad
    // turn against the slip angle rebuilt from the chassis state (body
    // velocity + yaw rate at each hub, minus the real wheel steer angle):
    // ratio -1.02 on both front wheels and -0.85 on both rear, i.e. equal
    // magnitude in degrees, opposite sign. The message publishes rad.
    msg.slip_angle[i] = w.lat_slip * M_PI / 180.0;
    // Effective mu, already multiplied by the road surface: a constant 0.70 of
    // the configured vehicle.physics.wheels[].tire_friction (1.05 against 1.5).
    msg.tire_friction[i] = w.tire_friction;
    msg.wheel_speed[i] = w.omega;
    // Built from omega and the wheel radius rather than passed through from
    // CARLA's long_slip, which contradicts the omega beside it: at a steady
    // 2.33 m/s cruise long_slip read +0.177 on a wheel turning 8.99 rad/s,
    // whose kinematic ratio is -0.034, and its per-wheel ordering does not
    // follow omega either. Below the speed floor the ratio has no meaning.
    msg.slip_ratio[i] =
        chassis_speed > kMinSlipRatioSpeed
            ? (w.omega * radius_m[i] - chassis_speed) / chassis_speed
            : 0.0;
    msg.normal_load[i] = w.tire_load;
    // Negated: CARLA's lat_force points the opposite way to the ROS body y
    // axis (CARLA is left-handed, +y right; ROS is right-handed, +y left) —
    // the same flip the per-wheel steer angles get below. Measured live over
    // two cornering runs: corr(+lat_force, m*ay) = -0.93, corr(-lat_force,
    // m*ay) = +0.93 with slope 0.86. lat_slip needs no flip, it already
    // agrees in sign with the chassis sideslip derived from /odom
    // (corr +0.71) and with m*ay (corr +0.92).
    msg.lateral_force[i] = -w.lat_force;
    // Published unmodified, and it is not the force the chassis receives in
    // either sign: corr(sum, m*ax) = +0.04 over 750 samples. It behaves as a
    // friction-capacity report — over 8268 wheel samples |long_force| sits
    // above 0.80*mu*Fz half the time and exactly at mu*Fz 15% of the time, and
    // cutting the engine's peak torque from 900 to 100 N*m (the car could then
    // no longer pass 0.87 m/s) left it pinned at 100% of the limit, so no
    // driveline or torque-curve change repairs it. At a steady cruise it reads
    // a saturated 703.4 N = 1.05 * 669.9 while the same wheel's lateral force
    // is 3.7 N, i.e. it is not coupled to the friction ellipse it appears to
    // saturate. A bridge-side model was tried and deleted (it was 7x off), so
    // consumers get the simulator's own number with the caveat stated in
    // TireForces.msg. long_force is exactly -torque/radius (identity to 1e-4 N
    // over 500 samples) and normalized_long_force is the same value over the
    // wheel's rest load, so neither is an independent reading.
    msg.longitudinal_force[i] = w.long_force;
    msg.wheel_torque[i] = w.torque;
  }

  tire_forces_pub_->publish(msg);
}

// Publish Vehicle State (lights / blinkers / steering)

void CarlaROS2Backend::publish_vehicle_state() {
  if (!vehicle_actor_) return;
  auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
  if (!v) return;

  using LS = carla::rpc::VehicleLightState::LightState;
  auto ls = static_cast<uint32_t>(v->GetLightState());
  auto has = [ls](LS flag) { return (ls & static_cast<uint32_t>(flag)) != 0; };

  // Steering mode (matches carla_msgs/srv/SetSteeringMode constants)
  auto steering_mode_name = [](int m) -> const char* {
    switch (m) {
      case 0:
        return "disable";
      case 1:
        return "front_ackerman";
      case 2:
        return "double_ackerman";
      case 3:
        return "crab_steer";
      case 4:
        return "front_parallel";
      case 5:
        return "double_parallel";
      case 6:
        return "go_to_home";
      case 7:
        return "calibration";
      default:
        return "unknown";
    }
  };

  nlohmann::json j = {
      {"timestamp", sim_now_epoch() * 1000.0},
      {"frame_id", "Vehicle_State"},
      {"lights",
       {{"position", has(LS::Position)},
        {"low_beam", has(LS::LowBeam)},
        {"high_beam", has(LS::HighBeam)},
        {"brake", has(LS::Brake)},
        {"reverse", has(LS::Reverse)},
        {"fog", has(LS::Fog)},
        {"interior", has(LS::Interior)},
        {"siren", has(LS::Special1)},
        {"special2", has(LS::Special2)}}},
      {"blinkers",
       {{"left", has(LS::LeftBlinker)},
        {"right", has(LS::RightBlinker)},
        {"hazard", has(LS::LeftBlinker) && has(LS::RightBlinker)}}},
      {"steering",
       {{"mode", steering_mode_name(steering_mode_)},
        {"mode_id", steering_mode_}}}};

  std_msgs::msg::String msg;
  msg.data = j.dump();
  vehicle_state_pub_->publish(msg);
}

void CarlaROS2Backend::publish_autonomous_mode() {
  std_msgs::msg::Bool msg;
  // Autonomous while no manual override; false once the user presses 'b'
  // in the GUI to force manual control.
  msg.data = !manual_control_override_;
  autonomous_mode_pub_->publish(msg);
}

void CarlaROS2Backend::publish_clock() {
  rosgraph_msgs::msg::Clock msg;
  msg.clock = epoch_to_stamp(sim_now_epoch());
  clock_pub_->publish(msg);
}

// Static TF

void CarlaROS2Backend::publish_static_transform(const std::string& parent,
                                                const std::string& child,
                                                float x, float y, float z,
                                                float roll, float pitch,
                                                float yaw) {
  if (parent.empty() || child.empty()) return;
  if (!static_tf_broadcaster_)
    static_tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node_);
  double qx, qy, qz, qw;
  carla_rpy_to_ros_quaternion(roll, pitch, yaw, qx, qy, qz, qw);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = epoch_to_stamp(sim_now_epoch());
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = -y;
  t.transform.translation.z = z;
  t.transform.rotation.x = qx;
  t.transform.rotation.y = qy;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;
  static_transforms_.push_back(t);
  static_tf_broadcaster_->sendTransform(static_transforms_);
}

void CarlaROS2Backend::publish_optical_transform(const std::string& parent,
                                                 const std::string& child) {
  if (!static_tf_broadcaster_) return;
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = epoch_to_stamp(sim_now_epoch());
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.rotation.x = -0.5;
  t.transform.rotation.y = 0.5;
  t.transform.rotation.z = -0.5;
  t.transform.rotation.w = 0.5;
  static_transforms_.push_back(t);
  static_tf_broadcaster_->sendTransform(static_transforms_);
}

// Cached CARLA state helpers

carla::rpc::VehiclePhysicsControl CarlaROS2Backend::physics(
    carla::client::Vehicle& v) {
  std::lock_guard<std::mutex> lk(physics_mutex_);
  // Fetched once, then served from the cache until a runtime friction/drag
  // command refreshes it. Copied out so callers never touch the cache while
  // apply_tire_friction/apply_drag_coefficient rewrites it.
  if (!physics_cached_) {
    cached_physics_ = v.GetPhysicsControl();
    physics_cached_ = true;
  }
  return cached_physics_;
}

void CarlaROS2Backend::apply_physics_preserving_motion(
    carla::client::Vehicle& v, const carla::rpc::VehiclePhysicsControl& pc) {
  // ApplyPhysicsControl recreates the vehicle's physics state, which zeroes
  // the rigid body's linear and angular velocity: measured on 0.9.16, a car at
  // 6 m/s reads exactly 0.00 m/s for ~22 ticks afterwards. Sampling the motion
  // first and writing it straight back turns that dead stop into a single-tick
  // dip of about 5%, so grip can be retuned while driving.
  const auto linear = v.GetVelocity();
  const auto angular = v.GetAngularVelocity();
  v.ApplyPhysicsControl(pc);
  v.SetTargetVelocity(linear);
  v.SetTargetAngularVelocity(angular);
}

void CarlaROS2Backend::apply_tire_friction(float friction) {
  if (!vehicle_actor_) return;
  auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
  if (!v) return;
  if (!std::isfinite(friction) || friction < 0.0f) {
    RCLCPP_WARN(node_->get_logger(),
                "[CarlaROS2Backend] Ignoring tire_friction=%.3f (must be "
                "finite and >= 0).",
                friction);
    return;
  }

  std::lock_guard<std::mutex> lk(physics_mutex_);
  if (std::fabs(friction - applied_tire_friction_) <= kFrictionEpsilon) {
    return;
  }
  // Re-read from the server instead of editing the cache: manual_control or
  // any other client may have changed physics since the last fetch, and
  // ApplyPhysicsControl writes the whole struct back.
  auto pc = v->GetPhysicsControl();
  auto wheels = pc.GetWheels();
  const std::string& drive_mode = drive_mode_;
  for (size_t i = 0; i < wheels.size(); ++i) {
    bool driven = (i < 2) ? (drive_mode != "RWD") : (drive_mode != "FWD");
    wheels[i].tire_friction =
        driven ? friction : CarlaVehicle::kNonDrivenTireFriction;
  }
  pc.SetWheels(wheels);
  apply_physics_preserving_motion(*v, pc);
  cached_physics_ = pc;
  physics_cached_ = true;
  applied_tire_friction_ = friction;

  // feedback/tire_forces reports CARLA's own per-wheel forces, so this physics
  // write is the only place grip is set — nothing downstream to keep in sync.
  RCLCPP_INFO(node_->get_logger(),
              "[CarlaROS2Backend] tire_friction = %.3f (drive_mode=%s).",
              friction, drive_mode.c_str());
}

void CarlaROS2Backend::apply_drag_coefficient(float drag) {
  if (!vehicle_actor_) return;
  auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
  if (!v) return;
  if (!std::isfinite(drag) || drag < 0.0f) {
    RCLCPP_WARN(
        node_->get_logger(),
        "[CarlaROS2Backend] Ignoring drag_coefficient=%.3f (must be finite "
        "and >= 0).",
        drag);
    return;
  }

  std::lock_guard<std::mutex> lk(physics_mutex_);
  if (std::fabs(drag - applied_drag_coefficient_) <= kFrictionEpsilon) {
    return;
  }
  auto pc = v->GetPhysicsControl();
  pc.drag_coefficient = drag;
  apply_physics_preserving_motion(*v, pc);
  cached_physics_ = pc;
  physics_cached_ = true;
  applied_drag_coefficient_ = drag;

  RCLCPP_INFO(node_->get_logger(), "[CarlaROS2Backend] drag_coefficient = %.3f",
              drag);
}

void CarlaROS2Backend::ensure_light_init(carla::client::Vehicle& v) {
  std::call_once(light_once_, [&] {
    light_state_.store(static_cast<uint32_t>(v.GetLightState()));
  });
}

void CarlaROS2Backend::set_light_bit(carla::client::Vehicle& v, uint32_t flag,
                                     bool on) {
  ensure_light_init(v);
  std::lock_guard<std::mutex> lk(
      light_set_mutex_);  // serialize read-modify-write
  uint32_t ls = light_state_.load();
  uint32_t next = on ? (ls | flag) : (ls & ~flag);
  if (next != ls) {  // only round-trip when it actually changes
    v.SetLightState(
        static_cast<carla::rpc::VehicleLightState::LightState>(next));
    light_state_.store(next);
  }
}

// Merged vehicle feedback (one snapshot)

void CarlaROS2Backend::publish_vehicle_feedback() {
  if (!vehicle_actor_) return;
  auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
  if (!v) return;

  // ONE snapshot: each Get* below is a blocking server RPC
  double t_rpc;
  t_rpc = PerfMonitor::tick();
  auto ctrl = v->GetControl();  // 1 RPC
  perf.record("rpc.GetControl", t_rpc);
  t_rpc = PerfMonitor::tick();
  auto telem = v->GetTelemetryData();  // 1 RPC
  perf.record("rpc.GetTelemetryData", t_rpc);
  // Frame the telemetry was served on. Read immediately AFTER the RPC returns,
  // never before: measured against a tick-aligned reference over 32 matched
  // samples, the snapshot taken after the call names the exact frame the data
  // came from every time, while the one taken before is a frame early in half
  // of them. GetSnapshot() is a local episode-state read, not an RPC.
  auto telem_snapshot = v->GetWorld().GetSnapshot();
  uint64_t telem_frame = static_cast<uint64_t>(telem_snapshot.GetFrame());
  double telem_epoch =
      sensor_sim_to_epoch(telem_snapshot.GetTimestamp().elapsed_seconds);
  const auto& phys = physics(*v);  // cached, no RPC
  ensure_light_init(*v);           // 1 RPC once, then cached
  uint32_t ls = light_state_;

  // Per-wheel steer angle (deg, CARLA sign) for the echo/JointState/Motors
  // telemetry below. In ackermann_drive, the vehicle is driven via
  // Vehicle::ApplyAckermannControl, and ctrl (= GetControl()) only reflects
  // the real command "if the ackermann control is inactive" — i.e. it's
  // stale here. Sample the real per-wheel angle via GetWheelSteerAngle
  // instead (4 RPCs, telemetry thread only, 10 Hz). Other modes still derive
  // it from ctrl.steer × max_steer_angle to avoid those RPCs, since ctrl is
  // authoritative there.
  float rawFL, rawFR, rawRL, rawRR;
  bool ctrl_valid_for_telemetry = control_mode_.source != "ackermann_drive";
  if (!ctrl_valid_for_telemetry) {
    t_rpc = PerfMonitor::tick();
    rawFL = v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::FL_Wheel);
    rawFR = v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::FR_Wheel);
    rawRL = v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::BL_Wheel);
    rawRR = v->GetWheelSteerAngle(carla::rpc::VehicleWheelLocation::BR_Wheel);
    perf.record("rpc.GetWheelSteerAngle", t_rpc);
  } else {
    rawFL = ctrl.steer *
            (phys.wheels.size() > 0 ? phys.wheels[0].max_steer_angle : 0.0f);
    rawFR = ctrl.steer *
            (phys.wheels.size() > 1 ? phys.wheels[1].max_steer_angle : 0.0f);
    rawRL = ctrl.steer *
            (phys.wheels.size() > 2 ? phys.wheels[2].max_steer_angle : 0.0f);
    rawRR = ctrl.steer *
            (phys.wheels.size() > 3 ? phys.wheels[3].max_steer_angle : 0.0f);
  }
  // Brake comes from GetTelemetryData(), which is sampled server-side after
  // CARLA's native Ackermann controller has run, so it is the brake actually
  // applied in every control mode. ctrl.brake (GetControl()) is only valid
  // while the ackermann controller is inactive and reads a flat 0 otherwise.
  float telemetry_brake = telem.brake;
  t_rpc = PerfMonitor::tick();
  auto velocity = v->GetVelocity();  // cheap (snapshot cache)
  perf.record("rpc.GetVelocity", t_rpc);
  t_rpc = PerfMonitor::tick();
  auto forward = v->GetTransform().GetForwardVector();
  perf.record("rpc.GetTransform", t_rpc);

  // Speed (forward-projected, signed)
  // Dot with the forward vector directly rather than taking the full 3D
  // velocity magnitude and re-signing it: magnitude folds in lateral slip
  // and vertical (suspension squat/bounce) velocity components, which
  // spikes under acceleration/cornering and was feeding false speed noise
  // into the ackermann speed PID and the tire-force rolling gate/brake_dir.
  double speed =
      velocity.x * forward.x + velocity.y * forward.y + velocity.z * forward.z;
  measured_speed_ms_.store(speed, std::memory_order_relaxed);
  {
    std_msgs::msg::Float32 m;
    m.data = static_cast<float>(speed);
    speed_pub_->publish(m);
  }

  // Steering echo + joints (ROS sign = negated CARLA angle)
  float sFL = -rawFL, sFR = -rawFR, sRL = -rawRL, sRR = -rawRR;
  {
    std_msgs::msg::Float32 e;
    e.data = static_cast<float>((sFL + sFR) / 2.0);
    steer_echo_pub_->publish(e);
  }
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = epoch_to_stamp(telem_epoch);  // frame-exact, see above
    js.name = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};
    js.position = {sFL, sFR, sRL, sRR};
    js.velocity = {0, 0, 0, 0};
    js.effort = {0, 0, 0, 0};
    steer_angles_pub_->publish(js);
  }

  // Motors JSON (raw per-wheel angles)
  if (telem.wheels.size() >= 4 && phys.wheels.size() >= 4) {
    float rFL = phys.wheels[0].radius / 100.0f,
          rFR = phys.wheels[1].radius / 100.0f;
    float rRL = phys.wheels[2].radius / 100.0f,
          rRR = phys.wheels[3].radius / 100.0f;
    auto wheel = [](const carla::rpc::WheelTelemetryData& w, float radius_m,
                    float steer_deg, float brake) {
      return nlohmann::json{{"speed_rpm", (w.omega * 60.0f) / (2.0f * M_PI)},
                            {"speed_mps", w.omega * radius_m},
                            {"steering_angle_deg", steer_deg},
                            {"steering_angle_rad", steer_deg * M_PI / 180.0f},
                            {"steering_angular_velocity", 0.0f},
                            {"steering_angular_velocity_radps", 0.0f},
                            {"brake_percentage", brake * 100.0f},
                            {"power", w.torque * std::abs(w.omega)},
                            {"torque", w.torque},
                            {"drive_motor_error", "OK"},
                            {"steering_motor_error", "OK"},
                            {"brake_motor_error", "OK"}};
    };
    nlohmann::json j = {
        {"timestamp", telem_epoch * 1000.0},
        {"frame_id", "Motors_Data"},
        {"front_left", wheel(telem.wheels[0], rFL, rawFL, telemetry_brake)},
        {"front_right", wheel(telem.wheels[1], rFR, rawFR, telemetry_brake)},
        {"back_left", wheel(telem.wheels[2], rRL, rawRL, telemetry_brake)},
        {"back_right", wheel(telem.wheels[3], rRR, rawRR, telemetry_brake)}};
    std_msgs::msg::String m;
    m.data = j.dump();
    motors_pub_->publish(m);
  }

  // Per-wheel tire state + forces (CARLA telemetry passthrough)
  publish_tire_forces(telem, telem_epoch, telem_frame);

  // Vehicle-state JSON (cached light bitmask)
  {
    using LS = carla::rpc::VehicleLightState::LightState;
    auto has = [ls](LS f) { return (ls & static_cast<uint32_t>(f)) != 0; };
    auto mode_name = [](int m) -> const char* {
      switch (m) {
        case 0:
          return "disable";
        case 1:
          return "front_ackerman";
        case 2:
          return "double_ackerman";
        case 3:
          return "crab_steer";
        case 4:
          return "front_parallel";
        case 5:
          return "double_parallel";
        case 6:
          return "go_to_home";
        case 7:
          return "calibration";
        default:
          return "unknown";
      }
    };
    nlohmann::json j = {
        {"timestamp", telem_epoch * 1000.0},
        {"frame_id", "Vehicle_State"},
        {"lights",
         {{"position", has(LS::Position)},
          {"low_beam", has(LS::LowBeam)},
          {"high_beam", has(LS::HighBeam)},
          {"brake", has(LS::Brake)},
          {"reverse", has(LS::Reverse)},
          {"fog", has(LS::Fog)},
          {"interior", has(LS::Interior)},
          {"siren", has(LS::Special1)},
          {"special2", has(LS::Special2)}}},
        {"blinkers",
         {{"left", has(LS::LeftBlinker)},
          {"right", has(LS::RightBlinker)},
          {"hazard", has(LS::LeftBlinker) && has(LS::RightBlinker)}}},
        {"steering",
         {{"mode", mode_name(steering_mode_)}, {"mode_id", steering_mode_}}}};
    std_msgs::msg::String m;
    m.data = j.dump();
    vehicle_state_pub_->publish(m);
  }

  // Autonomous-mode flag
  {
    std_msgs::msg::Bool m;
    m.data = !manual_control_override_;
    autonomous_mode_pub_->publish(m);
  }
}

// Vehicle Control

void CarlaROS2Backend::apply_vehicle_control() {
  if (!vehicle_actor_) return;
  auto v = boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
  if (!v) return;

  // In manual control mode, let manual_control.py handle everything
  // It will completely ignore autonomous commands AND the ROS brake topic.
  if (manual_control_override_) return;

  double t_rpc;

  // In autonomous mode, if brake is active, ALWAYS apply it.
  // Brake is the highest-priority command for autonomous mode.
  if (cmd_brake_) {
    t_rpc = PerfMonitor::tick();
    carla::rpc::VehicleControl ctrl = v->GetControl();
    perf.record("rpc.GetControl", t_rpc);
    ctrl.throttle = 0.0f;
    ctrl.brake = 1.0f;
    ctrl.reverse = false;
    ctrl.steering_mode = steering_mode_;  // GetControl doesn't round-trip it
    t_rpc = PerfMonitor::tick();
    v->ApplyControl(ctrl);
    perf.record("rpc.ApplyControl", t_rpc);
    last_steering_deg_ = 0.0;
    // Ensure PID integral doesn't wind up while braking
    rpm_pid_.integral = 0.0;
    // Brake lights on (cached — no per-tick GetLightState)
    set_light_bit(
        *v,
        static_cast<uint32_t>(carla::rpc::VehicleLightState::LightState::Brake),
        true);
    return;
  }

  // Note: cmd_brake_ case is handled above (early return with brake applied).
  // If we reach here, brake is NOT active.
  if (control_mode_.source == "ackermann_drive") {
    double target_speed = ack_speed_;
    if (control_mode_.max_velocity_ms > 0.0) {
      target_speed = std::clamp(target_speed, -control_mode_.max_velocity_ms,
                                control_mode_.max_velocity_ms);
    }

    // CARLA's own Ackermann controller (Vehicle::ApplyAckermannControl) runs
    // the speed/acceleration PID loops server-side — gains are set once via
    // ApplyAckermannControllerSettings in set_control_config(). This
    // replaces the client-side speed_pid_/steer_pid_ used previously.
    //
    // Steer sign: ROS ackermann_msgs is right-handed / REP-103 (positive =
    // left); CARLA inherits Unreal's left-handed frame (positive = right) and
    // its server-side Ackermann controller maps target steer to
    // VehicleControl.Steer with no sign flip. So the command must be negated
    // here - same convention as the vehicle_interface branch below, which
    // already does -cmd_steering_deg_ / max_steer_deg_. CARLA's own
    // AckermannControlSubscriber passing steering_angle through unchanged is
    // not a counter-example: it feeds a CARLA-frame DDS message, not a
    // REP-103 one.
    carla::rpc::VehicleAckermannControl ack_ctrl;
    ack_ctrl.steer = static_cast<float>(-ack_steering_angle_);
    ack_ctrl.steer_speed = static_cast<float>(ack_steering_angle_vel_);
    ack_ctrl.speed = static_cast<float>(target_speed);
    ack_ctrl.acceleration = static_cast<float>(ack_acceleration_);
    ack_ctrl.jerk = static_cast<float>(ack_jerk_);
    ack_ctrl.steering_mode = static_cast<uint8_t>(steering_mode_);

    t_rpc = PerfMonitor::tick();
    v->ApplyAckermannControl(ack_ctrl);
    perf.record("rpc.ApplyAckermannControl", t_rpc);

    last_steering_deg_ = ack_steering_angle_ * 180.0 / M_PI;

    // CARLA computes throttle/brake internally and never exposes them back
    // while the ackermann controller is active (see publish_vehicle_feedback
    // / publish_tire_forces for the same caveat), so the brake light is
    // approximated from commanded-vs-measured speed instead of a real brake
    // value.
    double measured = measured_speed_ms_.load(std::memory_order_relaxed);
    bool braking = std::abs(target_speed) + 0.2 < std::abs(measured);
    set_light_bit(
        *v,
        static_cast<uint32_t>(carla::rpc::VehicleLightState::LightState::Brake),
        braking);
    return;
  }

  t_rpc = PerfMonitor::tick();
  carla::rpc::VehicleControl ctrl = v->GetControl();
  perf.record("rpc.GetControl", t_rpc);

  {
    rclcpp::Time now = node_->now();

    auto velocity = v->GetVelocity();
    auto forward_vec = v->GetTransform().GetForwardVector();
    double forward_speed = velocity.x * forward_vec.x +
                           velocity.y * forward_vec.y +
                           velocity.z * forward_vec.z;

    double wheel_radius = kFallbackWheelRadius;
    const auto& physics = this->physics(*v);  // cached — no per-tick RPC
    if (!physics.wheels.empty()) {
      wheel_radius = physics.wheels[0].radius / 100.0;
    }

    double measured_rpm = (forward_speed / (2.0 * M_PI * wheel_radius)) * 60.0;

    double target_velocity_rpm = cmd_velocity_rpm_;
    if (control_mode_.max_velocity_ms > 0.0) {
      double max_rpm_cap =
          (control_mode_.max_velocity_ms / (2.0 * M_PI * wheel_radius)) * 60.0;
      target_velocity_rpm =
          std::clamp(target_velocity_rpm, -max_rpm_cap, max_rpm_cap);
    }

    bool target_reverse = (target_velocity_rpm < -0.1);
    bool moving_reverse = (measured_rpm < -0.1);

    if ((target_reverse && measured_rpm > 1.0) ||
        (!target_reverse && measured_rpm < -1.0 && target_velocity_rpm > 0.1)) {
      // Changing direction -> apply brake to stop first
      ctrl.brake = 1.0f;
      ctrl.throttle = 0.0f;
      ctrl.reverse = moving_reverse;
      rpm_pid_.integral = 0.0;
    } else {
      // Same direction or stopped
      ctrl.reverse = target_reverse;

      double target_mag = std::abs(target_velocity_rpm);
      double measured_mag = std::abs(measured_rpm);
      double error = target_mag - measured_mag;

      double pid_output = rpm_pid_.update(error, control_mode_.rpm_pid, now);

      if (control_mode_.hold_brake_at_standstill && target_mag < 1e-3 &&
          std::abs(forward_speed) < control_mode_.standstill_speed_ms) {
        // Same zero-setpoint hold as the ackermann branch above: error ~0
        // would otherwise command neutral (throttle 0, brake 0) and leave the
        // wheels free to keep spinning.
        ctrl.throttle = 0.0f;
        ctrl.brake = 1.0f;
        rpm_pid_.integral = 0.0;
      } else {
        ctrl.throttle = (pid_output > 0.0)
                            ? static_cast<float>(std::min(1.0, pid_output))
                            : 0.0f;
        ctrl.brake = (pid_output < -0.01)
                         ? static_cast<float>(std::min(1.0, -pid_output))
                         : 0.0f;
      }
    }
    ctrl.steer = static_cast<float>(std::clamp(
        -cmd_steering_deg_ / std::max(max_steer_deg_, 1.0), -1.0, 1.0));
  }
  ctrl.steering_mode = steering_mode_;  // GetControl doesn't round-trip it
  t_rpc = PerfMonitor::tick();
  v->ApplyControl(ctrl);
  perf.record("rpc.ApplyControl", t_rpc);
  last_steering_deg_ = cmd_steering_deg_;
  // Brake light from cache — only round-trips when the bit actually changes.
  set_light_bit(
      *v,
      static_cast<uint32_t>(carla::rpc::VehicleLightState::LightState::Brake),
      ctrl.brake > 0.5f);
}

// Helpers

void CarlaROS2Backend::carla_rpy_to_ros_quaternion(double r, double p, double y,
                                                   double& qx, double& qy,
                                                   double& qz, double& qw) {
  double roll = r * M_PI / 180.0;
  double pitch = -p * M_PI / 180.0;
  double yaw = -y * M_PI / 180.0;
  double cy = std::cos(yaw * .5), sy = std::sin(yaw * .5);
  double cp = std::cos(pitch * .5), sp = std::sin(pitch * .5);
  double cr = std::cos(roll * .5), sr = std::sin(roll * .5);
  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

void CarlaROS2Backend::shutdown() {
  RCLCPP_INFO(node_->get_logger(), "[CarlaROS2Backend] Shutdown complete.");
}

void CarlaROS2Backend::activate_publishers() {
  if (gps_pub_) gps_pub_->on_activate();
  if (gps_vel_pub_) gps_vel_pub_->on_activate();
  if (battery_pub_) battery_pub_->on_activate();
  if (imu_pub_) imu_pub_->on_activate();
  if (odom_pub_) odom_pub_->on_activate();
  if (boxes_pub_) boxes_pub_->on_activate();
  if (speed_pub_) speed_pub_->on_activate();
  if (steer_echo_pub_) steer_echo_pub_->on_activate();
  if (steer_angles_pub_) steer_angles_pub_->on_activate();
  if (motors_pub_) motors_pub_->on_activate();
  if (tire_forces_pub_) tire_forces_pub_->on_activate();
  if (vehicle_state_pub_) vehicle_state_pub_->on_activate();
  if (autonomous_mode_pub_) autonomous_mode_pub_->on_activate();
  if (clock_pub_) clock_pub_->on_activate();
  for (auto& [name, pubs] : camera_pubs_) {
    if (pubs.rgb) pubs.rgb->on_activate();
    if (pubs.info) pubs.info->on_activate();
  }
  for (auto& [name, pub] : lidar_pubs_) {
    if (pub) pub->on_activate();
  }
}

void CarlaROS2Backend::deactivate_publishers() {
  if (gps_pub_) gps_pub_->on_deactivate();
  if (gps_vel_pub_) gps_vel_pub_->on_deactivate();
  if (battery_pub_) battery_pub_->on_deactivate();
  if (imu_pub_) imu_pub_->on_deactivate();
  if (odom_pub_) odom_pub_->on_deactivate();
  if (boxes_pub_) boxes_pub_->on_deactivate();
  if (speed_pub_) speed_pub_->on_deactivate();
  if (steer_echo_pub_) steer_echo_pub_->on_deactivate();
  if (steer_angles_pub_) steer_angles_pub_->on_deactivate();
  if (motors_pub_) motors_pub_->on_deactivate();
  if (tire_forces_pub_) tire_forces_pub_->on_deactivate();
  if (vehicle_state_pub_) vehicle_state_pub_->on_deactivate();
  if (autonomous_mode_pub_) autonomous_mode_pub_->on_deactivate();
  if (clock_pub_) clock_pub_->on_deactivate();
  for (auto& [name, pubs] : camera_pubs_) {
    if (pubs.rgb) pubs.rgb->on_deactivate();
    if (pubs.info) pubs.info->on_deactivate();
  }
  for (auto& [name, pub] : lidar_pubs_) {
    if (pub) pub->on_deactivate();
  }
}

}  // namespace carla_telemetry
