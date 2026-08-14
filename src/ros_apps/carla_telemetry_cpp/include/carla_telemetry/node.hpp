#pragma once

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sim_manager_msgs/msg/component_health.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <vector>

#include "carla_telemetry/dynamic_props.hpp"
#include "carla_telemetry/npc_vehicles.hpp"
#include "carla_telemetry/ros2_backend.hpp"
#include "carla_telemetry/sensor_manager.hpp"
#include "carla_telemetry/sensors/battery.hpp"
#include "carla_telemetry/sensors/gps.hpp"
#include "carla_telemetry/sensors/ground_truth_boxes.hpp"
#include "carla_telemetry/sensors/imu.hpp"
#include "carla_telemetry/sensors/odometry.hpp"
#include "carla_telemetry/vehicle.hpp"
#include "carla_telemetry/walkers.hpp"

namespace carla_telemetry {

// Main ROS 2 node for CARLA telemetry.
// Direct port of CarlaTelemetryNode from node.py.
class CarlaTelemetryNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  CarlaTelemetryNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CarlaTelemetryNode() override;

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  void load_config(const std::string& path);
  void setup_vehicle();
  void setup_pedestrians();
  void setup_npc_vehicles();
  void setup_dynamic_props();
  void setup_sensors();
  void start_sensor_threads();
  void stop_sensor_threads();
  void shutdown();

  // Sensor thread loops
  void gps_loop(double hz);
  void battery_loop(double hz);
  void imu_loop(double hz);
  void odom_loop(double hz);
  void boxes_loop();
  void telemetry_loop(double hz);
  void control_loop(double hz);

  // YAML config (parsed once)
  YAML::Node config_;

  // Core components
  std::unique_ptr<CarlaVehicle> vehicle_;
  std::unique_ptr<CarlaROS2Backend> backend_;
  std::unique_ptr<CarlaSensorManager> sensor_mgr_;
  std::unique_ptr<CarlaGPS> gps_;
  std::unique_ptr<CarlaBattery> battery_;
  std::unique_ptr<CarlaIMU> imu_;
  std::unique_ptr<CarlaOdometry> odometry_;
  // odometry.follow_server_rate: when true the odom loop never dead-reckons —
  // it emits exactly one message per source sample, at that sample's own stamp.
  bool odom_follow_server_rate_ = false;
  std::unique_ptr<CarlaGroundTruthBoxes> ground_truth_boxes_;
  std::unique_ptr<CarlaWalkers> walkers_;
  std::unique_ptr<CarlaNpcVehicles> npc_vehicles_;
  std::unique_ptr<CarlaDynamicProps> dynamic_props_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      sim_manager_msgs::msg::ComponentHealth>>
      health_pub_;

  // Dedicated callback group for backend subscriptions and services, created
  // once in the constructor before the node is added to the executor.
  //
  // 1. Deadlock prevention: isolates blocking CARLA RPCs from the node's
  //    default callback group. The rclcpp_lifecycle services (change_state,
  //    get_state) execute in the default MutuallyExclusive group; if an RPC
  //    blocks waiting on a non-ticking server (the world doesn't tick while
  //    inactive), a shared group would deadlock subsequent
  //    `ros2 lifecycle set` calls.
  // 2. Executor safety: prevents callback group destruction while the
  //    executor is spinning. The backend is rebuilt on every configure
  //    transition, so a per-backend group would be destroyed dynamically,
  //    leaving a dangling raw guard-condition pointer in ROS 2 Humble
  //    rclcpp.
  rclcpp::CallbackGroup::SharedPtr entity_cb_group_;

  // Sensor threads (control_loop lives here too — see start_sensor_threads)
  std::vector<std::thread> sensor_threads_;
  std::atomic<bool> shutdown_{false};

  // Synchronizes ground-truth bounding boxes with sensor data streams.
  //
  // Ground-truth boxes are frame-locked to the simulation tick via
  // world.OnTick, mirroring the Listen mechanism used by camera and LiDAR
  // sensors:
  // 1. Lightweight callback: OnTick stays non-blocking. It updates
  //    ServerRate, gates execution on the shared world frame via
  //    sync_should_publish (the same decimation applied to the sensors), and
  //    forwards the frame's simulation timestamp to boxes_loop.
  // 2. Thread isolation: the blocking get_boxes RPC runs in boxes_loop rather
  //    than on the main tick thread, mirroring publish_loop's design.
  //
  // As a result, bounding boxes are emitted for the exact same world frames
  // as the LiDAR data, sharing a byte-identical timestamp with zero temporal
  // shift.
  size_t boxes_ontick_id_ = 0;
  bool boxes_ontick_registered_ = false;
  std::mutex boxes_mutex_;
  std::condition_variable boxes_cv_;
  double boxes_capture_time_ = 0.0;
  bool boxes_pending_ = false;

  // Set after on_cleanup; tells setup_vehicle to use the current CARLA
  // world instead of loading the map from config (reconfigure scenario).
  bool has_cleaned_up_ = false;

  double fixed_delta_ = 0.01667;
  bool dedicated_clients_enabled_ = false;
};

}  // namespace carla_telemetry
