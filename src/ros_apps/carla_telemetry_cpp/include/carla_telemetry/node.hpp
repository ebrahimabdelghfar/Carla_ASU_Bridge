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

/**
 * @brief Main ROS 2 node for CARLA telemetry.
 * Direct port of CarlaTelemetryNode from node.py.
 */
class CarlaTelemetryNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  /**
   * @brief Construct a new CarlaTelemetryNode object.
   *
   * @param options The node options
   */
  CarlaTelemetryNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CarlaTelemetryNode() override;

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Configure the node.
   *
   * @param state The node state
   * @return CallbackReturn The callback return
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Activate the node.
   *
   * @param state The node state
   * @return CallbackReturn The callback return
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Deactivate the node.
   *
   * @param state The node state
   * @return CallbackReturn The callback return
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Clean up the node.
   *
   * @param state The node state
   * @return CallbackReturn The callback return
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Shutdown the node.
   *
   * @param state The node state
   * @return CallbackReturn The callback return
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  /**
   * @brief Load the configuration.
   *
   * @param path The path to the configuration
   */
  void load_config(const std::string& path);
  /**
   * @brief Setup the vehicle.
   */
  void setup_vehicle();
  /**
   * @brief Setup the pedestrians.
   */
  void setup_pedestrians();
  /**
   * @brief Setup the npc vehicles.
   */
  void setup_npc_vehicles();
  /**
   * @brief Setup the dynamic props.
   */
  void setup_dynamic_props();
  /**
   * @brief Setup the sensors.
   */
  void setup_sensors();
  /**
   * @brief Start the sensor threads.
   */
  void start_sensor_threads();
  /**
   * @brief Stop the sensor threads.
   */
  void stop_sensor_threads();
  /**
   * @brief Shutdown the node.
   */
  void shutdown();

  // Sensor thread loops
  /**
   * @brief GPS loop.
   *
   * @param hz The update rate
   */
  void gps_loop(double hz);
  /**
   * @brief Battery loop.
   *
   * @param hz The update rate
   */
  void battery_loop(double hz);
  /**
   * @brief IMU loop.
   *
   * @param hz The update rate
   */
  void imu_loop(double hz);
  /**
   * @brief Odometry loop.
   *
   * @param hz The update rate
   */
  void odom_loop(double hz);
  /**
   * @brief Boxes loop.
   */
  void boxes_loop();
  /**
   * @brief Telemetry loop.
   *
   * @param hz The update rate
   */
  void telemetry_loop(double hz);
  /**
   * @brief Control loop.
   *
   * @param hz The update rate
   */
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

  /**
   * @brief Dedicated callback group for backend subscriptions and services.
   *
   * This is created once in the constructor (before the node is added to the
   * executor) and passed to every backend subscription and service.
   *
   * @note There are two critical architectural reasons for this lifecycle:
   *
   * 1. @b Deadlock @b prevention: Isolates blocking CARLA RPCs from the node's
   *    default callback group. The rclcpp_lifecycle services (change_state,
   *    get_state) execute in the default MutuallyExclusive group. If an RPC
   *    blocks waiting on a non-ticking server (since the world does not tick
   *    while inactive), a shared group would deadlock subsequent
   *    `ros2 lifecycle set` calls.
   *
   * 2. @b Executor @b safety: Prevents callback group destruction while the
   *    executor is spinning. Because the backend is rebuilt on every
   *    configure transition, a per-backend group would be destroyed
   *    dynamically, leaving a dangling raw guard-condition pointer in ROS 2
   *    Humble [rclcpp].
   */
  rclcpp::CallbackGroup::SharedPtr entity_cb_group_;

  // Sensor threads (control_loop lives here too — see start_sensor_threads)
  std::vector<std::thread> sensor_threads_;
  std::atomic<bool> shutdown_{false};

  /**
   * @brief Synchronizes ground-truth bounding boxes with sensor data streams.
   *
   * Ground-truth boxes are frame-locked to the simulation tick via
   * @c world.OnTick, mirroring the @c Listen mechanism used by camera and
   * LiDAR sensors.
   *
   * @details The synchronization follows a two-stage pipeline:
   *
   * 1. @b Lightweight @b callback: The @c OnTick callback remains non-blocking.
   *    It updates @c ServerRate, gates execution on the shared world frame via
   *    @c sync_should_publish (applying identical decimation to the sensors),
   *    and forwards the frame's simulation timestamp to @c boxes_loop.
   *
   * 2. @b Thread @b isolation: Blocking @c get_boxes RPCs execute
   * asynchronously within @c boxes_loop rather than on the main tick thread,
   * mirroring the architectural design of @c publish_loop.
   *
   * @note As a result, bounding boxes are emitted for the exact same world
   * frames as the LiDAR data, sharing a byte-identical timestamp with zero
   * temporal shift (0 ms).
   */
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
