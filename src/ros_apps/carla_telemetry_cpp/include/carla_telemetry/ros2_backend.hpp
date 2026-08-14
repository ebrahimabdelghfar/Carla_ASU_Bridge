#pragma once

#include <carla/client/Vehicle.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/VehiclePhysicsControl.h>
#include <carla/rpc/VehicleTelemetryData.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "carla_telemetry/types.hpp"

// ROS 2 message types
#include <carla/client/Actor.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <carla_msgs/srv/set_steering_mode.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sim_manager_msgs/msg/tire_forces.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace carla_telemetry {

class CarlaBattery;
class CarlaVehicle;

/**
 * @brief ROS 2 publisher + service layer for CARLA telemetry.
 * @details Direct port of CarlaROS2Backend from ros2_backend.py.
 */
class CarlaROS2Backend {
 public:
  /**
   * @brief Construct a new CarlaROS2Backend object.
   * @param node
   * @param topics_cfg Map of topic suffixes to ROS 2 topic names.
   * @param services_cfg Map of service suffixes to ROS 2 service names.
   * @param qos_cfg Map of QoS profiles to ROS 2 QoS settings.
   * @param ns ROS 2 namespace.
   * @param cb_group Callback group for the node.
   */
  CarlaROS2Backend(
      rclcpp_lifecycle::LifecycleNode* node,
      const std::unordered_map<std::string, std::string>& topics_cfg = {},
      const std::unordered_map<std::string, std::string>& services_cfg = {},
      const std::unordered_map<std::string, std::string>& qos_cfg = {},
      const std::string& ns = "sim",
      rclcpp::CallbackGroup::SharedPtr cb_group = nullptr);

  // ── Injectors ────────────────────────────────────────────────────
  /**
   * @brief PID controller configuration.
   */
  struct PIDConfig {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double max_integral = 0.0;
    double max_output = 0.0;
    PIDConfig() = default;
  };

  /**
   * @brief CARLA-native Ackermann controller PID gains (speed + acceleration
   * loops), applied once via Vehicle::ApplyAckermannControllerSettings.
   */
  struct AckermannControllerConfig {
    double speed_kp = 0.0;
    double speed_ki = 0.0;
    double speed_kd = 0.0;
    double accel_kp = 0.0;
    double accel_ki = 0.0;
    double accel_kd = 0.0;
    AckermannControllerConfig() = default;
  };

  /**
   * @brief Control mode configuration.
   */
  struct ControlModeConfig {
    std::string source = "existing_control_topic";
    PIDConfig steer_vel_pid;
    PIDConfig rpm_pid;
    AckermannControllerConfig ackermann_controller;
    double max_velocity_ms = 0.0;  // speed cap (m/s); <=0 disables the limiter

    /**
     * @brief Hold the brake when the commanded speed is zero and the vehicle
     * is already stopped, instead of coasting in neutral.
     *
     * @details A P-only speed controller cannot hold a standstill: with a zero
     * setpoint and a stationary vehicle the error is ~0, so it commands
     * throttle 0 @b and brake 0 — free-wheeling. Any wheel rotation (the
     * spawn drop, a slope, one stale throttle tick) then persists, because
     * @c damping_rate_zero_throttle_clutch_engaged /
     * @c ..._disengaged provide the only decay and may be configured to 0.
     * Set false to restore the old coast-in-neutral behaviour.
     */
    bool hold_brake_at_standstill = true;

    /// Speed (m/s) below which the vehicle counts as stopped for the hold.
    double standstill_speed_ms = 0.1;

    ControlModeConfig() = default;
  };

  /**
   * @brief Set the battery controller.
   * @param battery Pointer to the battery controller.
   */
  void set_battery_controller(CarlaBattery* battery) { battery_ = battery; }
  /**
   * @brief Set the vehicle.
   * @param vehicle Pointer to the vehicle.
   */
  void set_vehicle(CarlaVehicle* vehicle) { vehicle_ = vehicle; }
  /**
   * @brief Set the vehicle actor.
   * @param actor Pointer to the vehicle actor.
   */
  void set_vehicle_actor(carla::SharedPtr<carla::client::Actor> actor) {
    vehicle_actor_ = actor;
    if (vehicle_actor_) {
      auto v =
          boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_actor_);
      if (v) {
        auto ctrl = v->GetControl();
        // steering_mode_ defaults to 2 (DoubleAckerman)
        ctrl.steering_mode = steering_mode_;
        // Held stopped from the very first frame. A freshly spawned CARLA
        // vehicle has throttle 0 AND brake 0, so it free-wheels: the drop onto
        // its suspension can spin the wheels up, and with
        // damping_rate_zero_throttle_* at 0 nothing decays that rotation.
        // apply_vehicle_control()'s standstill hold takes over on the first
        // control tick; this closes the gap before it.
        ctrl.throttle = 0.0f;
        ctrl.brake = 1.0f;
        v->ApplyControl(ctrl);
      }
    }
  }
  /**
   * @brief Set the control configuration.
   * @param max_rpm Maximum RPM.
   * @param max_steer_deg Maximum steer angle in degrees.
   * @param mode_cfg Control mode configuration.
   */
  void set_control_config(double max_rpm, double max_steer_deg,
                          const ControlModeConfig& mode_cfg);
  /**
   * @brief Set which wheels the drivetrain turns ("AWD"/"FWD"/"RWD"), mirroring
   * vehicle.drive_mode. Used by set_tire_friction() to keep the non-driven
   * wheels at CarlaVehicle::kNonDrivenTireFriction when friction changes at
   * runtime.
   * @param drive_mode Drive mode string.
   */
  void set_drive_mode(const std::string& drive_mode) {
    drive_mode_ = drive_mode;
  }

  // ── Camera / LiDAR registration ──────────────────────────────────
  /**
   * @brief Register a camera.
   * @param name Camera name.
   * @param topic_rgb Topic for RGB images.
   * @param topic_info Topic for camera info.
   * @param qos_reliability QoS reliability.
   */
  void register_camera(const std::string& name,
                       const std::string& topic_rgb = "",
                       const std::string& topic_info = "",
                       const std::string& qos_reliability = "BEST_EFFORT");
  /**
   * @brief Register a LiDAR.
   * @param name LiDAR name.
   * @param topic Topic for LiDAR data.
   * @param qos_reliability QoS reliability.
   */
  void register_lidar(const std::string& name, const std::string& topic = "",
                      const std::string& qos_reliability = "BEST_EFFORT");

  // ── Publish methods ──────────────────────────────────────────────
  /**
   * @brief Publish GPS data.
   * @param gps GPS state.
   */
  void publish_gps(const GpsState& gps);
  /**
   * @brief Publish battery state.
   * @param bat Battery state.
   */
  void publish_battery(const BatteryState& bat);
  /**
   * @brief Publish IMU data.
   * @param imu IMU state.
   */
  void publish_imu(const ImuState& imu);
  /**
   * @brief Publish camera image.
   * @param data Camera data.
   */
  void publish_camera_image(CameraData& data);  // moves data.rgb into the msg
  /**
   * @brief Publish camera info.
   * @param data Camera data.
   */
  void publish_camera_info(const CameraData& data);
  /**
   * @brief Publish odometry data.
   * @param odom Odometry state.
   */
  void publish_odometry(const OdometryState& odom);
  /**
   * @brief Publish ground truth bounding boxes.
   * @param boxes Ground truth bounding boxes.
   */
  void publish_ground_truth_boxes(const GroundTruthBoxes& boxes);
  /**
   * @brief Publish point cloud data.
   * @param data Point cloud data.
   */
  void publish_point_cloud(const LidarData& data);
  /**
   * @brief Publish speed data.
   * @param speed_ms Speed in meters per second.
   */
  void publish_speed(double speed_ms);
  /**
   * @brief Publish steering angles.
   * @param steer_deg Steering angle in degrees.
   */
  void publish_steering_angles(
      double steer_deg = std::numeric_limits<double>::quiet_NaN());
  /**
   * @brief Publish motor data.
   */
  void publish_motors();
  /**
   * @brief Publish CARLA's own per-wheel slip, load, forces and torque from an
   * already-fetched telemetry snapshot. No CARLA RPCs, no analytic model.
   * @param telem Vehicle telemetry snapshot (from GetTelemetryData()).
   */
  void publish_tire_forces(const carla::rpc::VehicleTelemetryData& telem);
  /**
   * @brief Publish vehicle state data.
   */
  void publish_vehicle_state();
  /**
   * @brief Publish autonomous mode data.
   */
  void publish_autonomous_mode();
  /**
   * @brief Publish clock data.
   */
  void publish_clock();
  /**
   * @brief Publish vehicle feedback data.
   *
   * @details Merged feedback: takes ONE snapshot of the vehicle (one
   * GetControl, one GetTelemetryData, 4 GetWheelSteerAngle, cached physics +
   * light) and publishes speed + steering_angles + motors + vehicle_state +
   * autonomous_mode from it — instead of each publisher re-fetching. Runs on
   * the telemetry thread. Same data as the individual publishers.
   */
  void publish_vehicle_feedback();
  /**
   * @brief Publish static transform.
   * @param parent Parent frame.
   * @param child Child frame.
   * @param x x offset.
   * @param y y offset.
   * @param z z offset.
   * @param roll roll angle.
   * @param pitch pitch angle.
   * @param yaw yaw angle.
   */
  void publish_static_transform(const std::string& parent,
                                const std::string& child, float x, float y,
                                float z, float roll, float pitch, float yaw);
  /**
   * @brief Publish optical transform.
   * @param parent Parent frame.
   * @param child Child frame.
   */
  void publish_optical_transform(const std::string& parent,
                                 const std::string& child);

  // ── Control ──────────────────────────────────────────────────────
  /**
   * @brief Apply vehicle control.
   */
  void apply_vehicle_control();

  // ── Lifecycle ────────────────────────────────────────────────────
  /**
   * @brief Shutdown the backend.
   */
  void shutdown();
  /**
   * @brief Activate publishers.
   */
  void activate_publishers();
  /**
   * @brief Deactivate publishers.
   */
  void deactivate_publishers();

 private:
  /**
   * @brief Get the topic name.
   * @param suffix Topic suffix.
   * @return Topic name.
   */
  std::string topic(const std::string& suffix) const;

  /**
   * @brief Convert CARLA RPY to ROS quaternion.
   * @param roll_deg Roll angle in degrees.
   * @param pitch_deg Pitch angle in degrees.
   * @param yaw_deg Yaw angle in degrees.
   * @param qx Quaternion x.
   * @param qy Quaternion y.
   * @param qz Quaternion z.
   * @param qw Quaternion w.
   */
  static void carla_rpy_to_ros_quaternion(double roll_deg, double pitch_deg,
                                          double yaw_deg, double& qx,
                                          double& qy, double& qz, double& qw);

  rclcpp_lifecycle::LifecycleNode* node_;
  std::string namespace_;

  // ── Vehicle state ────────────────────────────────────────────────
  CarlaVehicle* vehicle_ = nullptr;
  carla::SharedPtr<carla::client::Actor> vehicle_actor_;
  CarlaBattery* battery_ = nullptr;
  double max_rpm_ = 150.0;
  double max_steer_deg_ = 16.0;
  ControlModeConfig control_mode_;
  /// Mirrors vehicle.drive_mode ("AWD"/"FWD"/"RWD").
  std::string drive_mode_ = "AWD";

  // PID states
  struct PIDState {
    double integral = 0.0;
    double error_prior = 0.0;
    rclcpp::Time last_time;
    bool initialized = false;
    /**
     * @brief Update the PID controller.
     * @param error Error value.
     * @param cfg PID configuration.
     * @param now Current time.
     * @return Updated output value.
     */
    double update(double error, const PIDConfig& cfg, rclcpp::Time now) {
      if (!initialized) {
        last_time = now;
        error_prior = error;
        initialized = true;
        return 0.0;
      }
      double dt = (now - last_time).seconds();
      last_time = now;
      if (dt <= 0.0) return 0.0;

      integral += error * dt;
      if (cfg.max_integral > 0.0) {
        integral = std::clamp(integral, -cfg.max_integral, cfg.max_integral);
      }

      double derivative = (error - error_prior) / dt;
      error_prior = error;

      double output = cfg.kp * error + cfg.ki * integral + cfg.kd * derivative;
      if (cfg.max_output > 0.0) {
        output = std::clamp(output, -cfg.max_output, cfg.max_output);
      }
      return output;
    }
  };
  PIDState steer_vel_pid_;
  PIDState rpm_pid_;

  // Commanded state
  double cmd_velocity_rpm_ = 0.0;
  double cmd_steering_deg_ = 0.0;
  bool cmd_brake_ = false;

  // Ackermann commanded state — forwarded as-is to CARLA's native
  // Vehicle::ApplyAckermannControl, which runs the speed/acceleration PID
  // loops server-side (see control.ackermann_controller in yaml).
  double ack_speed_ = 0.0;
  double ack_steering_angle_ = 0.0;
  double ack_steering_angle_vel_ = 0.0;
  double ack_acceleration_ = 0.0;
  double ack_jerk_ = 0.0;

  double last_steering_deg_ = 0.0;
  bool sim_running_ = true;

  // Forward-projected signed speed (m/s), sampled on the telemetry thread
  // (10 Hz, already computed there for speed_pub_ — no extra RPC). The
  // control loop (20 Hz) reads this lock-free to approximate the brake-light
  // state while ackermann_drive is active, since CARLA's native Ackermann
  // controller decides throttle/brake server-side and doesn't expose it back
  // (Vehicle::GetControl is only valid once the ackermann controller is
  // inactive).
  std::atomic<double> measured_speed_ms_{0.0};

  /**
   * @brief Locally cached CARLA simulation state to prevent per-tick blocking
   * RPCs.
   *
   * @details This state is accessed concurrently by the control timer and the
   * telemetry thread. To eliminate network overhead on execution hot paths,
   * simulation attributes are cached according to two optimization policies:
   *
   * 1. @b Lazily @b cached @b physics: Vehicle physics properties are fetched
   *    once on first use and then served from the cache. They are only
   *    re-fetched when this backend itself mutates them at runtime (see
   *    apply_tire_friction / apply_drag_coefficient), which refreshes the
   *    cache under @c physics_mutex_.
   *
   * 2. @b Light @b state @b mirroring: Vehicle light states are mirrored
   * locally so hot paths never call @c GetLightState directly. The remote
   *    @c SetLightState RPC is invoked only when a light bit explicitly
   *    transitions.
   *
   * @note Concurrency and thread-safety guarantees:
   *
   * - @b Initialization: Guarded by @c std::once_flag to guarantee thread-safe,
   *   one-time execution.
   * - @b Lock-free @b reads: The mirrored light state is managed via an
   *   @c std::atomic, enabling lock-free reads across threads.
   * - @b Mutex-guarded @b writes: Read-modify-write sequences that evaluate and
   *   invoke @c SetLightState are explicitly protected by a mutex to prevent
   *   race conditions.
   */
  std::mutex physics_mutex_;
  bool physics_cached_ = false;
  carla::rpc::VehiclePhysicsControl cached_physics_;

  std::once_flag light_once_;
  std::atomic<uint32_t> light_state_{0};
  std::mutex light_set_mutex_;

  /**
   * @brief Get the physics control.
   *
   * @details Returns a copy rather than a reference: the cache is mutable at
   * runtime (friction / drag topics), so handing out a reference would let the
   * telemetry thread read the struct while the subscription thread rewrites
   * it.
   *
   * @param v Vehicle.
   * @return Vehicle physics control.
   */
  carla::rpc::VehiclePhysicsControl physics(carla::client::Vehicle& v);

  /**
   * @brief Set the ground friction coefficient of all tires at runtime.
   *
   * @details Writes CARLA's per-wheel @c WheelPhysicsControl::tire_friction.
   * feedback/tire_forces reports CARLA's own per-wheel forces, so they follow
   * this change with nothing else to keep in sync. Non-driven wheels keep the
   * low-friction value that CarlaVehicle::apply_physics uses to emulate
   * FWD/RWD.
   *
   * @param friction Friction coefficient (>= 0).
   */
  void apply_tire_friction(float friction);

  /**
   * @brief Set the vehicle aerodynamic drag coefficient at runtime.
   * @param drag Drag coefficient (>= 0).
   */
  void apply_drag_coefficient(float drag);
  /**
   * @brief Ensure light state is initialized.
   * @param v Vehicle.
   */
  void ensure_light_init(carla::client::Vehicle& v);
  /**
   * @brief Set the light state.
   * @param v Vehicle.
   * @param flag Light flag.
   * @param on Light state.
   */
  void set_light_bit(carla::client::Vehicle& v, uint32_t flag, bool on);

  /**
   * @brief Persisted desired CARLA steering mode (e.g., value 2 for
   * DoubleAckerman).
   *
   * @details This value is locally cached and explicitly re-applied during
   * every control cycle to overcome an API limitation in the CARLA vehicle
   * controller.
   *
   * @note Why local persistence is required:
   *
   * 1. @b Missing @b round-trip: CARLA's @c GetControl() method does not
   *    preserve or return the @c steering_mode attribute. Fetching the current
   *    vehicle control struct yields an unpopulated or default steering state.
   *
   * 2. @b Prevention @b of @b clobbering: Because the attribute does not
   *    round-trip, the cached steering mode must be explicitly re-assigned on
   *    every invocation of @c ApplyControl(). Failing to re-apply this value
   *    during standard read-modify-write control loops would silently overwrite
   *    and clobber the active steering configuration.
   */
  int steering_mode_ =
      1;  // Default to DoubleAckerman when operating the carla first time

  // ── Publishers ───────────────────────────────────────────────────
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      gps_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr
      imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr
      odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr boxes_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
      speed_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
      steer_echo_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
      steer_angles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
      motors_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      sim_manager_msgs::msg::TireForces>::SharedPtr tire_forces_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
      vehicle_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
      autonomous_mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr
      clock_pub_;

  // Per-camera publishers: {name: {rgb, info}}
  struct CameraPubs {
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
        rgb;
    rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::CameraInfo>::SharedPtr info;
  };
  std::unordered_map<std::string, CameraPubs> camera_pubs_;

  // Per-lidar publishers
  std::unordered_map<std::string, rclcpp_lifecycle::LifecyclePublisher<
                                      sensor_msgs::msg::PointCloud2>::SharedPtr>
      lidar_pubs_;

  // ── Subscriptions ────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_rpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      ackermann_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr spawn_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sim_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sim_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tire_friction_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr drag_sub_;

  // ── Services ─────────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sim_start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sim_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_drain_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_drain_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_charge_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_charge_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr high_beams_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_lights_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_beams_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr left_blinker_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr right_blinker_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr siren_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr force_manual_control_srv_;
  rclcpp::Service<carla_msgs::srv::SetSteeringMode>::SharedPtr
      set_steering_mode_srv_;

  // Group owned by the node (created once, before add_node). Every
  // subscription/service here uses it, so the node's DEFAULT group stays free
  // for the lifecycle change_state/get_state services. NEVER create a group
  // in this class — the backend is rebuilt per configure. See node.hpp.
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  std::atomic<bool> manual_control_override_{false};
  ControlModeConfig original_control_mode_;

  // ── TF ───────────────────────────────────────────────────────────
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms_;

  // ── Config maps ──────────────────────────────────────────────────
  std::unordered_map<std::string, std::string> topics_cfg_;
  std::unordered_map<std::string, std::string> services_cfg_;
  std::unordered_map<std::string, std::string> qos_cfg_;
};

}  // namespace carla_telemetry
