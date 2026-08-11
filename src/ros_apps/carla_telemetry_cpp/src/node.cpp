#include "carla_telemetry/node.hpp"

#include <carla/client/WorldSnapshot.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "carla_telemetry/perf_monitor.hpp"
#include "carla_telemetry/sensors/sensor_clock.hpp"

using namespace std::chrono_literals;

namespace carla_telemetry {

CarlaTelemetryNode::CarlaTelemetryNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("micropilot_carla_bridge_node", options) {
  // Declare parameters here
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<std::string>("tire_model_config_file", "");
  this->declare_parameter<std::string>("open_manual_control", "");
  this->declare_parameter<std::string>("world_town", "");

  // One long-lived group for all backend entities — see node.hpp.
  entity_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

CarlaTelemetryNode::~CarlaTelemetryNode() { shutdown(); }

CarlaTelemetryNode::CallbackReturn CarlaTelemetryNode::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  // Reset shutdown flag so sensor threads can run after a cleanup→reconfigure
  // cycle
  shutdown_.store(false);

  std::string config_path;
  this->get_parameter("config_file", config_path);

  if (config_path.empty()) {
    // Try default location relative to workspace
    config_path = "config/carla_interface_config.yaml";
  }

  std::string tire_model_config_path;
  this->get_parameter("tire_model_config_file", tire_model_config_path);
  if (tire_model_config_path.empty()) {
    tire_model_config_path = "config/tire_model_config.yaml";
  }

  RCLCPP_INFO(this->get_logger(), "Loading config: %s", config_path.c_str());
  // A throw here would propagate out of the lifecycle callback and unwind the
  // executor, killing the process. Tear down the partial build and report
  // FAILURE instead, so the node stays in `unconfigured` and configure can be
  // retried without a stale half-built vehicle/backend in the way.
  try {
    load_config(config_path);
    load_tire_model_config(tire_model_config_path);
    setup_vehicle();
    setup_pedestrians();
    setup_npc_vehicles();
    setup_dynamic_props();
    setup_sensors();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Configure failed: %s", e.what());
    shutdown();
    return CallbackReturn::FAILURE;
  }

  // Create the health publisher
  if (!health_pub_) {
    health_pub_ =
        this->create_publisher<sim_manager_msgs::msg::ComponentHealth>(
            "/micropilot_system_manager_node/component_health", 1);
  }

  RCLCPP_INFO(this->get_logger(), "CarlaTelemetryNode configured.");
  return CallbackReturn::SUCCESS;
}

CarlaTelemetryNode::CallbackReturn CarlaTelemetryNode::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Activating...");
  if (!vehicle_ || !backend_) {
    RCLCPP_ERROR(this->get_logger(),
                 "No vehicle/backend; configure did not complete.");
    return CallbackReturn::FAILURE;
  }

  // Re-arm the loop flag: on_deactivate sets it to stop the sensor threads, so
  // without this a deactivate→activate cycle starts threads that exit at once.
  shutdown_.store(false);

  if (health_pub_) health_pub_->on_activate();

  if (backend_) {
    backend_->activate_publishers();
  }

  start_sensor_threads();

  return CallbackReturn::SUCCESS;
}

CarlaTelemetryNode::CallbackReturn CarlaTelemetryNode::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  // Join the control + sensor threads BEFORE the publishers go inactive.
  // start_sensor_threads appends to sensor_threads_, so leaving them running
  // here made every activate→deactivate→activate cycle stack a second copy of
  // every loop.
  stop_sensor_threads();
  if (health_pub_) health_pub_->on_deactivate();
  if (backend_) {
    backend_->deactivate_publishers();
  }
  return CallbackReturn::SUCCESS;
}

CarlaTelemetryNode::CallbackReturn CarlaTelemetryNode::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");
  shutdown();
  has_cleaned_up_ = true;  // next configure will use current world
  return CallbackReturn::SUCCESS;
}

CarlaTelemetryNode::CallbackReturn CarlaTelemetryNode::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  shutdown();
  return CallbackReturn::SUCCESS;
}

void CarlaTelemetryNode::load_config(const std::string& path) {
  config_ = YAML::LoadFile(path);

  auto carla = config_["carla"];
  fixed_delta_ = carla["fixed_delta_seconds"].as<double>(0.01667);
  auto ded_clients = carla["dedicated_clients"];
  dedicated_clients_enabled_ =
      ded_clients && ded_clients["enabled"].as<bool>(false);
  RCLCPP_INFO(this->get_logger(), "dedicated_clients.enabled = %s",
              dedicated_clients_enabled_ ? "true" : "false");
}

void CarlaTelemetryNode::load_tire_model_config(const std::string& path) {
  tire_model_config_ = YAML::LoadFile(path);
}

void CarlaTelemetryNode::setup_vehicle() {
  auto carla = config_["carla"];
  auto world = config_["world"];
  auto veh = config_["vehicle"];
  auto phys = veh["physics"];
  auto trans = veh["transmission"];

  CarlaVehicle::Config vc;
  vc.host = carla["host"].as<std::string>("localhost");
  vc.port = carla["port"].as<int>(2000);
  vc.timeout = carla["timeout"].as<double>(10.0);
  vc.sync_mode = carla["synchronous_mode"].as<bool>(true);
  vc.fixed_delta = fixed_delta_;
  // ROS2 parameter 'world_town' overrides the YAML config value when set.
  // This allows scenario_runner to change the map via the SetParameters
  // service before triggering a reconfigure, without touching the config file.
  std::string town_param;
  this->get_parameter("world_town", town_param);
  if (!town_param.empty()) {
    vc.town = town_param;
    RCLCPP_INFO(this->get_logger(), "Using world_town parameter override: %s",
                town_param.c_str());
  } else {
    vc.town = world["town"].as<std::string>("");
  }
  vc.use_current_world = has_cleaned_up_;
  if (world["spawn_point_coords"]) {
    auto coords = world["spawn_point_coords"];
    vc.use_coords = true;
    vc.spawn_x = coords["x"].as<float>(0.0f);
    vc.spawn_y = coords["y"].as<float>(0.0f);
    vc.spawn_z = coords["z"].as<float>(0.5f);
    vc.spawn_roll = coords["roll"].as<float>(0.0f);
    vc.spawn_pitch = coords["pitch"].as<float>(0.0f);
    vc.spawn_yaw = coords["yaw"].as<float>(0.0f);
  } else {
    vc.use_coords = false;
    vc.spawn_index = world["spawn_point_index"].as<int>(0);
  }
  vc.blueprint = veh["blueprint"].as<std::string>("vehicle.tesla.model3");
  vc.role_name = veh["role_name"].as<std::string>("");
  vc.generation = veh["generation"].as<std::string>("2");
  vc.color = veh["color"].as<std::string>("");

  // Manual control override from parameter
  std::string mc_override;
  this->get_parameter("open_manual_control", mc_override);
  if (mc_override == "true")
    vc.open_manual_control = true;
  else if (mc_override == "false")
    vc.open_manual_control = false;
  else
    vc.open_manual_control = carla["open_manual_control"].as<bool>(false);

  // manual_control display tuning (lighter render = more headroom for
  // LiDAR/cameras)
  auto mc_cfg = carla["manual_control"];
  if (mc_cfg) {
    vc.manual_control_res =
        mc_cfg["res"].as<std::string>(vc.manual_control_res);
    vc.manual_control_render_rate =
        mc_cfg["render_rate"].as<double>(vc.manual_control_render_rate);
  }

  CarlaVehicle::PhysicsConfig pc;
  pc.enabled = phys["enabled"].as<bool>(false);
  if (pc.enabled) {
    pc.mass = phys["mass"].as<float>(1500);
    pc.drag = phys["drag_coefficient"].as<float>(0.3f);
    if (phys["override_center_of_mass"]) {
      pc.override_center_of_mass =
          phys["override_center_of_mass"].as<bool>(false);
    } else {
      pc.override_center_of_mass = true;
    }
    auto com = phys["center_of_mass"];
    pc.com_x = com["x"].as<float>(0);
    pc.com_y = com["y"].as<float>(0);
    pc.com_z = com["z"].as<float>(0);
    pc.max_rpm = phys["max_rpm"].as<float>(6000);
    pc.moi = phys["moi"].as<float>(1);
    pc.damping_full = phys["damping_rate_full_throttle"].as<float>(0.15f);
    pc.damping_zero_eng =
        phys["damping_rate_zero_throttle_clutch_engaged"].as<float>(2.0f);
    pc.damping_zero_dis =
        phys["damping_rate_zero_throttle_clutch_disengaged"].as<float>(0.35f);
    pc.use_sweep = phys["use_sweep_wheel_collision"].as<bool>(false);

    for (auto pt : phys["torque_curve"])
      pc.torque_curve.emplace_back(pt[0].as<float>(), pt[1].as<float>());
    for (auto pt : phys["steering_curve"])
      pc.steering_curve.emplace_back(pt[0].as<float>(), pt[1].as<float>());

    pc.transmission_type = trans["type"].as<std::string>("automatic");
    pc.gear_switch_time = trans["gear_switch_time"].as<float>(0.5f);
    pc.clutch_strength = trans["clutch_strength"].as<float>(10);
    pc.final_ratio = trans["final_ratio"].as<float>(4);
    for (auto g : trans["forward_gears"])
      pc.forward_gears.push_back({g["ratio"].as<float>(),
                                  g["down_ratio"].as<float>(),
                                  g["up_ratio"].as<float>()});

    pc.drive_mode = veh["drive_mode"].as<std::string>("AWD");

    for (auto w : phys["wheels"]) {
      CarlaVehicle::PhysicsConfig::Wheel wh;
      wh.position = w["position"].as<std::string>("");
      wh.tire_friction = w["tire_friction"].as<float>(3.5f);
      wh.damping_rate = w["damping_rate"].as<float>(0.25f);
      wh.max_steer_angle = w["max_steer_angle"].as<float>(70);
      wh.radius = w["radius"].as<float>(36);
      wh.max_brake_torque = w["max_brake_torque"].as<float>(1500);
      wh.max_handbrake_torque = w["max_handbrake_torque"].as<float>(3000);
      pc.wheels.push_back(wh);
    }
  }

  vehicle_ = std::make_unique<CarlaVehicle>(vc, pc);

  // Setup ROS2 backend
  auto ros2_cfg = config_["ros2"];
  std::unordered_map<std::string, std::string> topics, services;
  if (ros2_cfg["topics"])
    for (auto it = ros2_cfg["topics"].begin(); it != ros2_cfg["topics"].end();
         ++it)
      topics[it->first.as<std::string>()] = it->second.as<std::string>();
  if (ros2_cfg["services"])
    for (auto it = ros2_cfg["services"].begin();
         it != ros2_cfg["services"].end(); ++it)
      services[it->first.as<std::string>()] = it->second.as<std::string>();
  std::string ns = ros2_cfg["namespace"].as<std::string>("sim");

  std::unordered_map<std::string, std::string> qos_cfg;
  if (config_["gps"] && config_["gps"]["qos_reliability"])
    qos_cfg["gps"] = config_["gps"]["qos_reliability"].as<std::string>();
  if (config_["battery"] && config_["battery"]["qos_reliability"])
    qos_cfg["battery"] =
        config_["battery"]["qos_reliability"].as<std::string>();
  if (config_["imu"] && config_["imu"]["qos_reliability"])
    qos_cfg["imu"] = config_["imu"]["qos_reliability"].as<std::string>();
  if (config_["odometry"] && config_["odometry"]["qos_reliability"])
    qos_cfg["odometry"] =
        config_["odometry"]["qos_reliability"].as<std::string>();
  if (config_["ground_truth_boxes"] &&
      config_["ground_truth_boxes"]["qos_reliability"])
    qos_cfg["ground_truth_boxes"] =
        config_["ground_truth_boxes"]["qos_reliability"].as<std::string>();

  backend_ = std::make_unique<CarlaROS2Backend>(this, topics, services, qos_cfg,
                                                ns, entity_cb_group_);
  backend_->set_vehicle_actor(vehicle_->actor());

  auto ctrl = config_["control"];
  CarlaROS2Backend::ControlModeConfig mode_cfg;
  if (ctrl["source"]) {
    mode_cfg.source = ctrl["source"].as<std::string>("existing_control_topic");
  }

  auto load_pid = [](YAML::Node pid_node, CarlaROS2Backend::PIDConfig& pid) {
    if (pid_node) {
      pid.kp = pid_node["kp"].as<double>(0.0);
      pid.ki = pid_node["ki"].as<double>(0.0);
      pid.kd = pid_node["kd"].as<double>(0.0);
      pid.max_integral = pid_node["max_integral"].as<double>(0.0);
      pid.max_output = pid_node["max_output"].as<double>(0.0);
    }
  };
  if (ctrl["steer_vel_pid"])
    load_pid(ctrl["steer_vel_pid"], mode_cfg.steer_vel_pid);
  if (ctrl["rpm_pid"]) load_pid(ctrl["rpm_pid"], mode_cfg.rpm_pid);

  if (auto ac = ctrl["ackermann_controller"]) {
    mode_cfg.ackermann_controller.speed_kp = ac["speed_kp"].as<double>(0.0);
    mode_cfg.ackermann_controller.speed_ki = ac["speed_ki"].as<double>(0.0);
    mode_cfg.ackermann_controller.speed_kd = ac["speed_kd"].as<double>(0.0);
    mode_cfg.ackermann_controller.accel_kp = ac["accel_kp"].as<double>(0.0);
    mode_cfg.ackermann_controller.accel_ki = ac["accel_ki"].as<double>(0.0);
    mode_cfg.ackermann_controller.accel_kd = ac["accel_kd"].as<double>(0.0);
  }

  mode_cfg.max_velocity_ms = ctrl["max_velocity_kmh"].as<double>(0.0) / 3.6;
  mode_cfg.hold_brake_at_standstill =
      ctrl["hold_brake_at_standstill"].as<bool>(true);
  mode_cfg.standstill_speed_ms = ctrl["standstill_speed_ms"].as<double>(0.1);

  backend_->set_control_config(ctrl["max_rpm"].as<double>(150),
                               ctrl["max_steer_deg"].as<double>(70), mode_cfg);

  // ── Ground-truth tire model (Magic Formula + motor coefficient) ──────
  CarlaROS2Backend::TireModelConfig tire_model_cfg;
  tire_model_cfg.drive_mode = veh["drive_mode"].as<std::string>("AWD");
  auto tm = tire_model_config_["tire_model"];
  for (auto w : tm["magic_formula"]["wheels"]) {
    CarlaROS2Backend::TireModelConfig::Wheel mfw;
    mfw.position = w["position"].as<std::string>("");
    mfw.B = w["B"].as<double>(10.0);
    mfw.C = w["C"].as<double>(1.9);
    mfw.E = w["E"].as<double>(0.97);
    tire_model_cfg.wheels.push_back(mfw);
  }
  auto motor = tm["motor"];
  tire_model_cfg.Cm1 = motor["Cm1"].as<double>(8000.0);
  tire_model_cfg.Cm2 = motor["Cm2"].as<double>(150.0);
  tire_model_cfg.Cr0 = motor["Cr0"].as<double>(60.0);
  tire_model_cfg.Cd = motor["Cd"].as<double>(1.2);
  backend_->set_tire_model_config(tire_model_cfg);
}

void CarlaTelemetryNode::setup_pedestrians() {
  auto ped_cfg = config_["pedestrians"];
  if (!ped_cfg || !ped_cfg["enabled"].as<bool>(false)) return;

  CarlaWalkers::Config wcfg;
  wcfg.enabled = true;
  wcfg.count = ped_cfg["count"].as<int>(0);
  if (ped_cfg["blueprints"] && ped_cfg["blueprints"].IsSequence()) {
    for (auto bp : ped_cfg["blueprints"]) {
      wcfg.blueprints.push_back(bp.as<std::string>());
    }
  } else if (ped_cfg["blueprints"] && ped_cfg["blueprints"].IsScalar()) {
    wcfg.blueprints.push_back(ped_cfg["blueprints"].as<std::string>());
  } else {
    wcfg.blueprints.push_back("walker.pedestrian.*");
  }
  wcfg.speed = ped_cfg["speed"].as<float>(1.4f);
  wcfg.sync_mode = config_["carla"]["synchronous_mode"].as<bool>(true);

  walkers_ = std::make_unique<CarlaWalkers>(vehicle_->world(),
                                            vehicle_->client(), wcfg);
}

void CarlaTelemetryNode::setup_npc_vehicles() {
  auto npc_cfg = config_["npc_vehicles"];
  if (!npc_cfg || !npc_cfg["enabled"].as<bool>(false)) return;

  CarlaNpcVehicles::Config vcfg;
  vcfg.enabled = true;
  vcfg.count = npc_cfg["count"].as<int>(0);
  if (npc_cfg["blueprints"] && npc_cfg["blueprints"].IsSequence()) {
    for (auto bp : npc_cfg["blueprints"]) {
      vcfg.blueprints.push_back(bp.as<std::string>());
    }
  } else if (npc_cfg["blueprints"] && npc_cfg["blueprints"].IsScalar()) {
    vcfg.blueprints.push_back(npc_cfg["blueprints"].as<std::string>());
  } else {
    vcfg.blueprints.push_back("vehicle.*");
  }
  vcfg.autopilot = npc_cfg["autopilot"].as<bool>(true);
  vcfg.tm_port = npc_cfg["tm_port"].as<int>(8000);
  vcfg.max_speed_kmh = npc_cfg["max_speed_kmh"].as<float>(0.0f);
  vcfg.speed_difference_pct = npc_cfg["speed_difference_pct"].as<float>(0.0f);
  vcfg.distance_to_leading =
      npc_cfg["distance_to_leading_vehicle"].as<float>(5.0f);
  vcfg.auto_lane_change = npc_cfg["auto_lane_change"].as<bool>(false);
  vcfg.hybrid_physics = npc_cfg["hybrid_physics"].as<bool>(true);
  vcfg.hybrid_physics_radius =
      npc_cfg["hybrid_physics_radius"].as<float>(70.0f);
  vcfg.sync_mode = config_["carla"]["synchronous_mode"].as<bool>(true);

  carla::geom::Location ego_loc;
  if (vehicle_ && vehicle_->actor()) {
    ego_loc = vehicle_->actor()->GetLocation();
  }
  npc_vehicles_ =
      std::make_unique<CarlaNpcVehicles>(vehicle_->world(), vcfg, ego_loc);
}

void CarlaTelemetryNode::setup_dynamic_props() {
  auto props_cfg = config_["dynamic_props"];
  if (!props_cfg || !props_cfg["enabled"].as<bool>(false)) return;

  CarlaDynamicProps::Config dcfg;
  dcfg.enabled = true;
  if (props_cfg["spawn_on_roads"]) {
    dcfg.spawn_on_roads = props_cfg["spawn_on_roads"].as<bool>(true);
  }
  dcfg.count = props_cfg["count"].as<int>(0);
  dcfg.max_distance = props_cfg["max_distance"].as<float>(10.0f);
  if (props_cfg["min_distance_from_ego"]) {
    dcfg.min_distance_from_ego =
        props_cfg["min_distance_from_ego"].as<float>(8.0f);
  }
  if (props_cfg["prop_to_prop_distance"]) {
    dcfg.prop_to_prop_distance =
        props_cfg["prop_to_prop_distance"].as<float>(2.5f);
  }
  dcfg.spawn_height = props_cfg["spawn_height"].as<float>(1.0f);

  if (props_cfg["blueprints"] && props_cfg["blueprints"].IsSequence()) {
    for (auto bp : props_cfg["blueprints"]) {
      dcfg.blueprints.push_back(bp.as<std::string>());
    }
  } else if (props_cfg["blueprints"] && props_cfg["blueprints"].IsScalar()) {
    dcfg.blueprints.push_back(props_cfg["blueprints"].as<std::string>());
  } else {
    dcfg.blueprints.push_back("static.prop.*");
  }

  carla::geom::Location ego_loc;
  if (vehicle_ && vehicle_->actor()) {
    ego_loc = vehicle_->actor()->GetLocation();
  }

  dynamic_props_ =
      std::make_unique<CarlaDynamicProps>(vehicle_->world(), dcfg, ego_loc);
}

void CarlaTelemetryNode::setup_sensors() {
  auto& world = vehicle_->world();
  auto actor = vehicle_->actor();

  // GPS
  auto gps_cfg = config_["gps"];
  CarlaGPS::Config gc;
  gc.update_rate = gps_cfg["update_rate"].as<double>(10);
  gc.gps_xy_random_walk = gps_cfg["gps_xy_random_walk"].as<double>(2);
  gc.gps_z_random_walk = gps_cfg["gps_z_random_walk"].as<double>(4);
  gc.gps_correlation_time = gps_cfg["gps_correlation_time"].as<double>(60);
  gc.gps_xy_noise_density = gps_cfg["gps_xy_noise_density"].as<double>(2e-4);
  gc.gps_z_noise_density = gps_cfg["gps_z_noise_density"].as<double>(4e-4);
  gc.gps_vxy_noise_density = gps_cfg["gps_vxy_noise_density"].as<double>(0.2);
  gc.gps_vz_noise_density = gps_cfg["gps_vz_noise_density"].as<double>(0.4);
  auto sp = gps_cfg["spawn_point"];
  gc.sp_x = sp["x"].as<float>(1);
  gc.sp_y = sp["y"].as<float>(0);
  gc.sp_z = sp["z"].as<float>(2.8f);
  gc.sp_roll = sp["roll"].as<float>(0);
  gc.sp_pitch = sp["pitch"].as<float>(0);
  gc.sp_yaw = sp["yaw"].as<float>(0);

  auto gc_node = config_["global_coordinates"];
  double origin_lat = gc_node["latitude"].as<double>(0);
  double origin_lon = gc_node["longitude"].as<double>(0);
  double origin_alt = gc_node["altitude"].as<double>(0);
  gps_ = std::make_unique<CarlaGPS>(world, actor, gc, origin_lat, origin_lon,
                                    origin_alt);

  // Battery
  auto bat = config_["battery"];
  battery_ = std::make_unique<CarlaBattery>(
      bat["consumption_mode"].as<std::string>("constant"),
      bat["voltage"].as<double>(12.6),
      bat["open_circuit_voltage_constant_coef"].as<double>(12.6),
      bat["open_circuit_voltage_linear_coef"].as<double>(-2.8),
      bat["capacity"].as<double>(10), bat["initial_charge"].as<double>(10),
      bat["resistance"].as<double>(0.061),
      bat["smooth_current_tau"].as<double>(2), bat["power_load"].as<double>(50),
      bat["power_per_speed"].as<double>(15),
      bat["start_draining"].as<bool>(true),
      bat["enable_recharge"].as<bool>(false),
      bat["charging_time"].as<double>(1), bat["update_rate"].as<double>(10),
      bat["ambient_temperature"].as<double>(25.0));
  backend_->set_battery_controller(battery_.get());

  // IMU
  auto imu_cfg = config_["imu"];
  if (imu_cfg["enabled"].as<bool>(true)) {
    auto isp = imu_cfg["spawn_point"];
    imu_ = std::make_unique<CarlaIMU>(
        world, actor, imu_cfg["frame_id"].as<std::string>("imu_link"),
        isp["x"].as<float>(0), isp["y"].as<float>(0), isp["z"].as<float>(0),
        isp["roll"].as<float>(0), isp["pitch"].as<float>(0),
        isp["yaw"].as<float>(0));
  }

  // Odometry
  auto odom_cfg = config_["odometry"];
  if (odom_cfg["enabled"].as<bool>(true)) {
    auto gc_node = config_["global_coordinates"];
    double origin_lat = gc_node["latitude"].as<double>(0.0);
    double origin_lon = gc_node["longitude"].as<double>(0.0);
    double origin_alt = gc_node["altitude"].as<double>(0.0);

    OdometryNoiseConfig noise_cfg;
    if (odom_cfg["noise"]) {
      auto noise = odom_cfg["noise"];
      noise_cfg.enabled = noise["enabled"].as<bool>(false);
      noise_cfg.pos_stddev_x = noise["pos_stddev_x"].as<double>(0.0);
      noise_cfg.pos_stddev_y = noise["pos_stddev_y"].as<double>(0.0);
      noise_cfg.pos_stddev_z = noise["pos_stddev_z"].as<double>(0.0);
      noise_cfg.ori_stddev_roll = noise["ori_stddev_roll"].as<double>(0.0);
      noise_cfg.ori_stddev_pitch = noise["ori_stddev_pitch"].as<double>(0.0);
      noise_cfg.ori_stddev_yaw = noise["ori_stddev_yaw"].as<double>(0.0);
      noise_cfg.vel_stddev_x = noise["vel_stddev_x"].as<double>(0.0);
      noise_cfg.vel_stddev_y = noise["vel_stddev_y"].as<double>(0.0);
      noise_cfg.vel_stddev_z = noise["vel_stddev_z"].as<double>(0.0);
      noise_cfg.ang_vel_stddev_x = noise["ang_vel_stddev_x"].as<double>(0.0);
      noise_cfg.ang_vel_stddev_y = noise["ang_vel_stddev_y"].as<double>(0.0);
      noise_cfg.ang_vel_stddev_z = noise["ang_vel_stddev_z"].as<double>(0.0);
    }

    bool gnss_use_noise = odom_cfg["gnss_use_noise"].as<bool>(true);

    odometry_ = std::make_unique<CarlaOdometry>(
        odom_cfg["frame_id"].as<std::string>("odom"),
        odom_cfg["child_frame_id"].as<std::string>("base_link"),
        odom_cfg["update_rate"].as<double>(20),
        odom_cfg["broadcast_tf"].as<bool>(false),
        odom_cfg["mode"].as<std::string>("standard"), origin_lat, origin_lon,
        origin_alt, gnss_use_noise, noise_cfg);
  }

  // Ground-truth 3D bounding boxes
  auto boxes_cfg = config_["ground_truth_boxes"];
  if (!boxes_cfg || boxes_cfg["enabled"].as<bool>(true)) {
    std::string boxes_frame =
        boxes_cfg ? boxes_cfg["frame_id"].as<std::string>("base_link")
                  : "base_link";
    CarlaGroundTruthBoxes::RangeWindow boxes_window;
    if (boxes_cfg && boxes_cfg["range_window"]) {
      auto rw = boxes_cfg["range_window"];
      boxes_window.max_range = rw["max_range"].as<double>(0.0);
      boxes_window.min_range = rw["min_range"].as<double>(0.0);
      boxes_window.horizontal_fov = rw["horizontal_fov"].as<double>(360.0);
      boxes_window.z_min = rw["z_min"].as<double>(0.0);
      boxes_window.z_max = rw["z_max"].as<double>(0.0);
    }
    ground_truth_boxes_ =
        std::make_unique<CarlaGroundTruthBoxes>(boxes_frame, boxes_window);
  }

  // Cameras
  sensor_mgr_ = std::make_unique<CarlaSensorManager>();
  auto tf_cfg = config_["tf"];
  bool broadcast_sensor_tf = tf_cfg["broadcast_sensor_tf"].as<bool>(true);
  std::string base_frame = tf_cfg["base_frame_id"].as<std::string>("base_link");

  // Connection params for dedicated camera clients (multi-client mode).
  auto carla_node = config_["carla"];
  std::string cc_host = carla_node["host"].as<std::string>("localhost");
  int cc_port = carla_node["port"].as<int>(2000);
  double cc_timeout = carla_node["timeout"].as<double>(10.0);
  std::string cc_role = config_["vehicle"]["role_name"].as<std::string>("hero");
  carla::rpc::ActorId ego_id = vehicle_->actor()->GetId();

  for (auto cam_cfg : config_["cameras"]) {
    if (!cam_cfg["enabled"].as<bool>(true)) continue;
    CarlaCamera::Config cc;
    cc.name = cam_cfg["name"].as<std::string>("camera");
    cc.frame_id = cam_cfg["frame_id"].as<std::string>(cc.name);
    cc.type = cam_cfg["type"].as<std::string>("sensor.camera.rgb");
    cc.update_rate = cam_cfg["update_rate"].as<float>(30);
    cc.image_size_x = cam_cfg["image_size_x"].as<int>(640);
    cc.image_size_y = cam_cfg["image_size_y"].as<int>(480);
    cc.fov = cam_cfg["fov"].as<float>(90);
    cc.topic_rgb = cam_cfg["topic_rgb"].as<std::string>("");
    cc.topic_camera_info = cam_cfg["topic_camera_info"].as<std::string>("");
    std::string qos = cam_cfg["qos_reliability"].as<std::string>("BEST_EFFORT");
    auto csp = cam_cfg["spawn_point"];
    cc.sp_x = csp["x"].as<float>(0);
    cc.sp_y = csp["y"].as<float>(0);
    cc.sp_z = csp["z"].as<float>(1.5f);
    cc.sp_roll = csp["roll"].as<float>(0);
    cc.sp_pitch = csp["pitch"].as<float>(0);
    cc.sp_yaw = csp["yaw"].as<float>(0);

    backend_->register_camera(cc.frame_id, cc.topic_rgb, cc.topic_camera_info,
                              qos);

    if (broadcast_sensor_tf) {
      backend_->publish_static_transform(base_frame, cc.frame_id, cc.sp_x,
                                         cc.sp_y, cc.sp_z, cc.sp_roll,
                                         cc.sp_pitch, cc.sp_yaw);
      backend_->publish_optical_transform(cc.frame_id,
                                          cc.frame_id + "_optical");
    }

    if (dedicated_clients_enabled_) {
      // Dedicated client per camera: own TCP stream + io_context.
      auto cam_client = std::make_unique<CameraClient>(
          cc_host, cc_port, cc_timeout, ego_id, cc_role, cc);
      if (cam_client->ok()) {
        sensor_mgr_->add_camera_client(std::move(cam_client));
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Camera '%s' dedicated client failed to init; skipping.",
                    cc.name.c_str());
      }
    } else {
      // Shared main-client path (original behavior).
      auto camera = std::make_unique<CarlaCamera>(world, actor, cc);
      sensor_mgr_->add_camera(std::move(camera));
    }
  }

  // Third-person chase view (mirrors the manual_control default view).
  // A single RGB camera; built exactly like a `cameras` entry so it gets its
  // own dedicated client when dedicated_clients.enabled is true.
  if (auto tpv_cfg = config_["third_person_view"];
      tpv_cfg && tpv_cfg["enabled"].as<bool>(false)) {
    CarlaCamera::Config cc;
    cc.name = tpv_cfg["name"].as<std::string>("third_person_view");
    cc.frame_id = tpv_cfg["frame_id"].as<std::string>(cc.name);
    cc.type = tpv_cfg["type"].as<std::string>("sensor.camera.rgb");
    cc.update_rate = tpv_cfg["update_rate"].as<float>(20);
    cc.image_size_x = tpv_cfg["image_size_x"].as<int>(960);
    cc.image_size_y = tpv_cfg["image_size_y"].as<int>(540);
    cc.fov = tpv_cfg["fov"].as<float>(90);
    cc.topic_rgb = tpv_cfg["topic_rgb"].as<std::string>("");
    cc.topic_camera_info = tpv_cfg["topic_camera_info"].as<std::string>("");
    std::string qos = tpv_cfg["qos_reliability"].as<std::string>("BEST_EFFORT");
    auto csp = tpv_cfg["spawn_point"];
    cc.sp_x = csp["x"].as<float>(-5.5f);
    cc.sp_y = csp["y"].as<float>(0);
    cc.sp_z = csp["z"].as<float>(2.8f);
    cc.sp_roll = csp["roll"].as<float>(0);
    cc.sp_pitch = csp["pitch"].as<float>(8.0f);
    cc.sp_yaw = csp["yaw"].as<float>(0);

    backend_->register_camera(cc.frame_id, cc.topic_rgb, cc.topic_camera_info,
                              qos);

    if (broadcast_sensor_tf) {
      backend_->publish_static_transform(base_frame, cc.frame_id, cc.sp_x,
                                         cc.sp_y, cc.sp_z, cc.sp_roll,
                                         cc.sp_pitch, cc.sp_yaw);
      backend_->publish_optical_transform(cc.frame_id,
                                          cc.frame_id + "_optical");
    }

    if (dedicated_clients_enabled_) {
      auto cam_client = std::make_unique<CameraClient>(
          cc_host, cc_port, cc_timeout, ego_id, cc_role, cc);
      if (cam_client->ok()) {
        sensor_mgr_->add_camera_client(std::move(cam_client));
      } else {
        RCLCPP_WARN(
            this->get_logger(),
            "third_person_view dedicated client failed to init; skipping.");
      }
    } else {
      auto camera = std::make_unique<CarlaCamera>(world, actor, cc);
      sensor_mgr_->add_camera(std::move(camera));
    }
  }

  // LiDARs
  for (auto lidar_cfg : config_["lidars"]) {
    if (!lidar_cfg["enabled"].as<bool>(true)) continue;

    // Depth-camera LiDAR: GPU depth ring reprojected to a point cloud.
    if (lidar_cfg["lidar_type"].as<std::string>("rotary") == "depth") {
      CarlaDepthLidar::Config dc;
      dc.name = lidar_cfg["name"].as<std::string>("depth_lidar");
      dc.frame_id = lidar_cfg["frame_id"].as<std::string>(dc.name);
      dc.topic_point_cloud = lidar_cfg["topic_point_cloud"].as<std::string>("");
      dc.update_rate = lidar_cfg["update_rate"].as<float>(10);
      dc.num_cameras = lidar_cfg["num_cameras"].as<int>(6);
      dc.image_size_x = lidar_cfg["image_size_x"].as<int>(400);
      dc.image_size_y = lidar_cfg["image_size_y"].as<int>(300);
      dc.fov = lidar_cfg["fov"].as<float>(0.0f);
      dc.range = lidar_cfg["range"].as<float>(100.0f);
      dc.min_range = lidar_cfg["min_range"].as<float>(0.5f);
      dc.point_stride = lidar_cfg["point_stride"].as<int>(2);
      auto dsp = lidar_cfg["spawn_point"];
      dc.sp_x = dsp["x"].as<float>(0);
      dc.sp_y = dsp["y"].as<float>(0);
      dc.sp_z = dsp["z"].as<float>(2.4f);
      dc.sp_roll = dsp["roll"].as<float>(0);
      dc.sp_pitch = dsp["pitch"].as<float>(0);
      dc.sp_yaw = dsp["yaw"].as<float>(0);
      std::string dqos =
          lidar_cfg["qos_reliability"].as<std::string>("BEST_EFFORT");

      backend_->register_lidar(dc.name, dc.topic_point_cloud, dqos);
      if (broadcast_sensor_tf) {
        backend_->publish_static_transform(base_frame, dc.frame_id, dc.sp_x,
                                           dc.sp_y, dc.sp_z, dc.sp_roll,
                                           dc.sp_pitch, dc.sp_yaw);
      }
      if (dedicated_clients_enabled_) {
        auto dclient = std::make_unique<DepthLidarClient>(
            cc_host, cc_port, cc_timeout, ego_id, cc_role, dc);
        if (dclient->ok()) {
          sensor_mgr_->add_depth_lidar_client(std::move(dclient));
        } else {
          RCLCPP_WARN(
              this->get_logger(),
              "DepthLiDAR '%s' dedicated client failed to init; skipping.",
              dc.name.c_str());
        }
      } else {
        auto dl = std::make_unique<CarlaDepthLidar>(world, actor, dc);
        sensor_mgr_->add_depth_lidar(std::move(dl));
      }
      continue;
    }

    CarlaLidar::Config lc;
    lc.name = lidar_cfg["name"].as<std::string>("lidar");
    lc.frame_id = lidar_cfg["frame_id"].as<std::string>(lc.name);
    lc.lidar_type = lidar_cfg["lidar_type"].as<std::string>("rotary");
    lc.use_compute = lidar_cfg["use_compute"].as<bool>(false);
    lc.update_rate = lidar_cfg["update_rate"].as<float>(20);
    lc.channels = lidar_cfg["channels"].as<int>(64);
    lc.range = lidar_cfg["range"].as<float>(85);
    lc.points_per_second = lidar_cfg["points_per_second"].as<int>(600000);
    lc.rotation_frequency = lidar_cfg["rotation_frequency"].as<float>(20);
    if (lc.rotation_frequency <= 0.0f) {
      float derived =
          std::max(lc.update_rate, static_cast<float>(1.0 / fixed_delta_));
      lc.rotation_frequency = derived;
      RCLCPP_INFO(this->get_logger(),
                  "Lidar '%s' rotation_frequency auto-derived to %.3f Hz "
                  "(max of update_rate=%.1f, 1/fixed_delta=%.1f)",
                  lc.name.c_str(), derived, lc.update_rate, 1.0 / fixed_delta_);
    }
    lc.upper_fov = lidar_cfg["upper_fov"].as<float>(10);
    lc.lower_fov = lidar_cfg["lower_fov"].as<float>(-30);
    lc.atmosphere_attenuation_rate =
        lidar_cfg["atmosphere_attenuation_rate"].as<float>(0.004f);
    lc.dropoff_general_rate =
        lidar_cfg["dropoff_general_rate"].as<float>(0.45f);
    lc.dropoff_intensity_limit =
        lidar_cfg["dropoff_intensity_limit"].as<float>(0.8f);
    lc.dropoff_zero_intensity =
        lidar_cfg["dropoff_zero_intensity"].as<float>(0.4f);
    lc.horizontal_fov = lidar_cfg["horizontal_fov"].as<float>(120);
    lc.vertical_fov = lidar_cfg["vertical_fov"].as<float>(30);
    lc.topic_point_cloud = lidar_cfg["topic_point_cloud"].as<std::string>("");
    std::string qos =
        lidar_cfg["qos_reliability"].as<std::string>("BEST_EFFORT");
    auto lsp = lidar_cfg["spawn_point"];
    lc.sp_x = lsp["x"].as<float>(0);
    lc.sp_y = lsp["y"].as<float>(0);
    lc.sp_z = lsp["z"].as<float>(2.4f);
    lc.sp_roll = lsp["roll"].as<float>(0);
    lc.sp_pitch = lsp["pitch"].as<float>(0);
    lc.sp_yaw = lsp["yaw"].as<float>(0);

    backend_->register_lidar(lc.name, lc.topic_point_cloud, qos);

    if (broadcast_sensor_tf) {
      backend_->publish_static_transform(base_frame, lc.frame_id, lc.sp_x,
                                         lc.sp_y, lc.sp_z, lc.sp_roll,
                                         lc.sp_pitch, lc.sp_yaw);
    }

    if (dedicated_clients_enabled_) {
      // Dedicated client per LiDAR: own TCP stream + io_context.
      auto lidar_client = std::make_unique<LidarClient>(
          cc_host, cc_port, cc_timeout, ego_id, cc_role, lc);
      if (lidar_client->ok()) {
        sensor_mgr_->add_lidar_client(std::move(lidar_client));
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "LiDAR '%s' dedicated client failed to init; skipping.",
                    lc.name.c_str());
      }
    } else {
      // Shared main-client path (original behavior).
      auto lidar = std::make_unique<CarlaLidar>(world, actor, lc);
      sensor_mgr_->add_lidar(std::move(lidar));
    }
  }

  // GPS static TF
  if (broadcast_sensor_tf) {
    backend_->publish_static_transform(base_frame, "gps_link", gc.sp_x, gc.sp_y,
                                       gc.sp_z, gc.sp_roll, gc.sp_pitch,
                                       gc.sp_yaw);
  }

  // Wheels static TF
  if (broadcast_sensor_tf && vehicle_ && vehicle_->actor()) {
    auto v =
        boost::dynamic_pointer_cast<carla::client::Vehicle>(vehicle_->actor());
    if (v) {
      auto phys = v->GetPhysicsControl();
      auto veh_trans = v->GetTransform();
      std::vector<std::string> wheel_names = {"wheel_FL", "wheel_FR",
                                              "wheel_RL", "wheel_RR"};
      for (size_t i = 0; i < std::min(phys.wheels.size(), wheel_names.size());
           ++i) {
        auto& w = phys.wheels[i];
        // w.position is in centimeters (world coordinates)
        // Convert to meters first
        carla::geom::Vector3D wheel_pos_m(w.position.x / 100.0f,
                                          w.position.y / 100.0f,
                                          w.position.z / 100.0f);

        // Transform world coordinates to vehicle local coordinates
        veh_trans.InverseTransformPoint(wheel_pos_m);

        // Convert left-handed CARLA coordinates to right-handed ROS coordinates
        float x = wheel_pos_m.x;
        float y = -wheel_pos_m.y;
        float z = wheel_pos_m.z;
        backend_->publish_static_transform(base_frame, wheel_names[i], x, y, z,
                                           0.0f, 0.0f, 0.0f);
      }
    }
  }

  // Start camera + lidar sensor threads
  sensor_mgr_->start_all(backend_.get());
}

// ---------------------------------------------------------------------------
// Dedicated sensor threads (matching Python _start_sensor_thread pattern)
// ---------------------------------------------------------------------------

void CarlaTelemetryNode::start_sensor_threads() {
  auto cfg = config_;

  double gps_hz = cfg["gps"]["update_rate"].as<double>(10.0);
  double battery_hz = cfg["battery"]["update_rate"].as<double>(10.0);
  double imu_hz = cfg["imu"]["update_rate"].as<double>(50.0);
  double odom_hz = cfg["odometry"]["update_rate"].as<double>(20.0);
  double boxes_hz =
      cfg["ground_truth_boxes"]
          ? cfg["ground_truth_boxes"]["update_rate"].as<double>(10.0)
          : 10.0;
  double telem_hz = cfg["telemetry"]
                        ? cfg["telemetry"]["update_rate"].as<double>(20.0)
                        : 20.0;

  // Control tick first: in sync mode it is what advances the world the sensor
  // loops read from.
  sensor_threads_.emplace_back(&CarlaTelemetryNode::control_loop, this,
                               1.0 / fixed_delta_);

  if (gps_) {
    sensor_threads_.emplace_back(&CarlaTelemetryNode::gps_loop, this, gps_hz);
  }
  if (battery_) {
    sensor_threads_.emplace_back(&CarlaTelemetryNode::battery_loop, this,
                                 battery_hz);
  }
  if (imu_) {
    sensor_threads_.emplace_back(&CarlaTelemetryNode::imu_loop, this, imu_hz);
  }
  if (odometry_) {
    sensor_threads_.emplace_back(&CarlaTelemetryNode::odom_loop, this, odom_hz);
  }
  if (ground_truth_boxes_) {
    // Frame-lock boxes to the world tick — the same sync path the camera/LiDAR
    // use. OnTick is the Listen analogue: it fires once per world frame. The
    // callback stays cheap (no CARLA RPC): feed the shared server-rate
    // estimator, decimate with sync_should_publish on the shared frame number
    // (so boxes select the SAME frames as the LiDAR), and stamp with that
    // frame's sim time via sensor_sim_to_epoch (0 ms cross-sensor shift). The
    // blocking get_boxes runs in boxes_loop, off the tick thread.
    const double boxes_rate = boxes_hz;
    boxes_ontick_id_ = vehicle_->world().OnTick(
        [this, boxes_rate](carla::client::WorldSnapshot snap) {
          uint64_t frame = static_cast<uint64_t>(snap.GetFrame());
          ServerRate::instance().report(frame);
          if (!sync_should_publish(frame, boxes_rate)) return;
          double cap = sensor_sim_to_epoch(snap.GetTimestamp().elapsed_seconds);
          {
            std::lock_guard<std::mutex> lk(boxes_mutex_);
            boxes_capture_time_ = cap;
            boxes_pending_ = true;
          }
          boxes_cv_.notify_one();
        });
    boxes_ontick_registered_ = true;
    sensor_threads_.emplace_back(&CarlaTelemetryNode::boxes_loop, this);
  }
  // Vehicle feedback thread (always on) — decoupled from the control timer.
  sensor_threads_.emplace_back(&CarlaTelemetryNode::telemetry_loop, this,
                               telem_hz);
}

void CarlaTelemetryNode::stop_sensor_threads() {
  shutdown_.store(true);
  // Detach the box OnTick before joining so no further frames wake the worker.
  if (boxes_ontick_registered_ && vehicle_) {
    try {
      vehicle_->world().RemoveOnTick(boxes_ontick_id_);
    } catch (...) {
    }
    boxes_ontick_registered_ = false;
  }
  boxes_cv_.notify_all();  // wake boxes_loop out of its wait
  for (auto& t : sensor_threads_) {
    if (t.joinable()) t.join();
  }
  sensor_threads_.clear();
}

void CarlaTelemetryNode::gps_loop(double hz) {
  using clock = std::chrono::steady_clock;
  double period = 1.0 / hz;
  auto next_time = clock::now();
  auto prev_time = std::chrono::steady_clock::now();

  while (!shutdown_.load() && rclcpp::ok()) {
    auto now = clock::now();
    if (next_time > now) {
      std::this_thread::sleep_until(next_time);
    }
    next_time += std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(period));
    if (next_time < clock::now()) {
      next_time = clock::now() + std::chrono::duration_cast<clock::duration>(
                                     std::chrono::duration<double>(period));
    }

    if (!vehicle_ || !vehicle_->is_running()) continue;

    try {
      auto wall_now = clock::now();
      double dt = std::chrono::duration<double>(wall_now - prev_time).count();
      prev_time = wall_now;

      auto vel = vehicle_->actor()->GetVelocity();
      double capture_time =
          sim_now_epoch();  // shared sim clock (see sensor_clock.hpp)
      auto state = gps_->update(vel.x, -vel.y, vel.z, dt);
      if (state && backend_) {
        state->capture_time = capture_time;
        backend_->publish_gps(*state);
      }
    } catch (const std::exception& e) {
      // Silently continue on CARLA errors
    }
  }
}

void CarlaTelemetryNode::battery_loop(double hz) {
  using clock = std::chrono::steady_clock;
  double period = 1.0 / hz;
  auto next_time = clock::now();
  auto prev_time = clock::now();

  while (!shutdown_.load() && rclcpp::ok()) {
    auto now = clock::now();
    if (next_time > now) {
      std::this_thread::sleep_until(next_time);
    }
    next_time += std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(period));
    if (next_time < clock::now()) {
      next_time = clock::now() + std::chrono::duration_cast<clock::duration>(
                                     std::chrono::duration<double>(period));
    }

    if (!vehicle_ || !vehicle_->is_running()) continue;

    try {
      auto wall_now = clock::now();
      double dt = std::chrono::duration<double>(wall_now - prev_time).count();
      prev_time = wall_now;

      auto vel = vehicle_->actor()->GetVelocity();
      double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
      double capture_time =
          sim_now_epoch();  // shared sim clock (see sensor_clock.hpp)
      auto state = battery_->update(speed, dt);
      state.capture_time = capture_time;
      if (backend_) {
        backend_->publish_battery(state);
      }
    } catch (const std::exception& e) {
      // Silently continue on CARLA errors
    }
  }
}

void CarlaTelemetryNode::imu_loop(double hz) {
  using clock = std::chrono::steady_clock;
  double period = 1.0 / hz;
  auto next_time = clock::now();

  while (!shutdown_.load() && rclcpp::ok()) {
    auto now = clock::now();
    if (next_time > now) {
      std::this_thread::sleep_until(next_time);
    }
    next_time += std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(period));
    if (next_time < clock::now()) {
      next_time = clock::now() + std::chrono::duration_cast<clock::duration>(
                                     std::chrono::duration<double>(period));
    }

    if (!vehicle_ || !vehicle_->is_running()) continue;

    try {
      double capture_time =
          sim_now_epoch();  // shared sim clock (see sensor_clock.hpp)
      auto state = imu_->get_state();
      if (state && backend_) {
        state->capture_time = capture_time;
        backend_->publish_imu(*state);
      }
    } catch (const std::exception& e) {
      // Silently continue on CARLA errors
    }
  }
}

void CarlaTelemetryNode::odom_loop(double hz) {
  using clock = std::chrono::steady_clock;
  double period = 1.0 / hz;
  auto next_time = clock::now();

  while (!shutdown_.load() && rclcpp::ok()) {
    auto now = clock::now();
    if (next_time > now) {
      std::this_thread::sleep_until(next_time);
    }
    next_time += std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(period));
    if (next_time < clock::now()) {
      next_time = clock::now() + std::chrono::duration_cast<clock::duration>(
                                     std::chrono::duration<double>(period));
    }

    if (!vehicle_ || !vehicle_->is_running()) continue;

    try {
      // Stamp on the shared sim clock (same as the sensors), NOT wall time:
      // the server runs slower than realtime, so wall-stamped odom/TF would
      // drift ahead of the sim-stamped point clouds and the ego pose used to
      // place the cloud would be from a later instant -> spatial shift.
      double capture_time = sim_now_epoch();
      auto state = odometry_->get_state(vehicle_->actor(), gps_.get());
      state.capture_time = capture_time;
      if (backend_) {
        backend_->publish_odometry(state);
      }
    } catch (const std::exception& e) {
      // Silently continue on CARLA errors
    }
  }
}

void CarlaTelemetryNode::boxes_loop() {
  // Event-driven, frame-locked to the world tick (mirrors the sensor
  // publish_loop): block until the OnTick callback selects a world frame, then
  // read + publish the boxes stamped with THAT frame's sim time. No wall timer
  // — the boxes emit the SAME frames as the LiDAR at the same stamp.
  while (!shutdown_.load() && rclcpp::ok()) {
    double capture_time;
    {
      std::unique_lock<std::mutex> lk(boxes_mutex_);
      boxes_cv_.wait(lk, [this] { return boxes_pending_ || shutdown_.load(); });
      if (shutdown_.load()) break;
      boxes_pending_ = false;
      capture_time = boxes_capture_time_;
    }

    if (!vehicle_ || !vehicle_->is_running()) continue;

    try {
      auto state =
          ground_truth_boxes_->get_boxes(vehicle_->world(), vehicle_->actor());
      state.capture_time = capture_time;
      if (backend_) {
        backend_->publish_ground_truth_boxes(state);
      }
    } catch (const std::exception& e) {
      // Silently continue on CARLA errors
    }
  }
}

// Vehicle feedback on its own thread: one snapshot of the vehicle per cycle
// (motors/steering/state/speed/autonomous) at telemetry.update_rate. Kept off
// the control timer + ROS executor so its blocking CARLA RPCs never stall them.
void CarlaTelemetryNode::telemetry_loop(double hz) {
  using clock = std::chrono::steady_clock;
  double period = 1.0 / hz;
  auto next_time = clock::now();

  while (!shutdown_.load() && rclcpp::ok()) {
    auto now = clock::now();
    if (next_time > now) {
      std::this_thread::sleep_until(next_time);
    }
    next_time += std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(period));
    if (next_time < clock::now()) {
      next_time = clock::now() + std::chrono::duration_cast<clock::duration>(
                                     std::chrono::duration<double>(period));
    }

    if (!vehicle_ || !vehicle_->is_running()) continue;

    try {
      auto t = PerfMonitor::tick();
      if (backend_) backend_->publish_vehicle_feedback();
      perf.record("telemetry.loop", t);
    } catch (const std::exception& e) {
      // Silently continue on CARLA errors
    }
  }
}

// ---------------------------------------------------------------------------
// Control loop — only ticks world + applies control (sensors on own threads)
// ---------------------------------------------------------------------------

// Runs on its own thread, NOT on an rclcpp timer: the timer lived in a callback
// group that had to be created per configure, which corrupted the Humble
// executor wait set and latched change_state (bug-005/bug-006). As a plain
// thread it is joined by stop_sensor_threads() before shutdown() destroys
// vehicle_/backend_, so no lock is needed to keep it off a dead actor.
void CarlaTelemetryNode::control_loop(double hz) {
  using clock = std::chrono::steady_clock;
  double period = 1.0 / hz;
  auto next_time = clock::now();

  while (!shutdown_.load() && rclcpp::ok()) {
    auto now = clock::now();
    if (next_time > now) {
      std::this_thread::sleep_until(next_time);
    }
    next_time += std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(period));
    if (next_time < clock::now()) {
      next_time = clock::now() + std::chrono::duration_cast<clock::duration>(
                                     std::chrono::duration<double>(period));
    }

    auto t = PerfMonitor::tick();

    try {
      // Tick world (no-op in async mode) + apply control only. All vehicle
      // feedback (speed/steering/motors/state/autonomous) moved to
      // telemetry_loop so its RPC-heavy reads never block this control tick
      // nor the ROS executor.
      if (vehicle_) vehicle_->tick();
      // Pump client-side pedestrian navigation (no-op in sync / when no
      // walkers). In async the world is never Tick()'d here, so this is the
      // only thing that advances the walker crowd — without it walkers freeze.
      if (walkers_) walkers_->navigation_tick();
      if (backend_) {
        backend_->apply_vehicle_control();
        backend_->publish_clock();
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "CARLA tick/control error (transient during world reload): %s",
          e.what());
    }

    if (health_pub_ && health_pub_->is_activated()) {
      sim_manager_msgs::msg::ComponentHealth msg;
      msg.component_id = this->get_name();
      if (!vehicle_ || !vehicle_->is_running()) {
        msg.status = msg.ERROR;
        msg.message = "carla server not running";
      } else {
        msg.status = msg.OK;
        msg.message = "";
      }
      msg.stamp = this->now();
      health_pub_->publish(msg);
    }

    perf.record("node.timer", t);
  }
}

void CarlaTelemetryNode::shutdown() {
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  if (vehicle_) vehicle_->stop_manual_control();
  // Sets shutdown_ and joins the control + sensor loops, so nothing is still
  // touching vehicle_/backend_ when they are destroyed below.
  stop_sensor_threads();
  if (sensor_mgr_) {
    sensor_mgr_->destroy_all();
    sensor_mgr_.reset();
  }
  if (gps_) {
    gps_->destroy();
    gps_.reset();
  }
  if (imu_) {
    imu_->destroy();
    imu_.reset();
  }
  if (odometry_) {
    odometry_.reset();
  }
  if (backend_) {
    backend_->shutdown();
  }
  if (npc_vehicles_) {
    npc_vehicles_->destroy();
    npc_vehicles_.reset();
  }
  if (walkers_) {
    walkers_->destroy();
    walkers_.reset();
  }
  if (dynamic_props_) {
    dynamic_props_->destroy();
    dynamic_props_.reset();
  }
  if (vehicle_) {
    vehicle_->destroy();
    vehicle_.reset();
  }
  if (battery_) {
    battery_.reset();
  }
  if (health_pub_) {
    health_pub_.reset();
  }
  RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
}

}  // namespace carla_telemetry
