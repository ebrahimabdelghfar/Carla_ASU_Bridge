#pragma once

#include <memory>
#include <string>
#include <vector>

#include "carla_telemetry/sensors/camera.hpp"
#include "carla_telemetry/sensors/depth_lidar.hpp"
#include "carla_telemetry/sensors/lidar.hpp"
#include "carla_telemetry/sensors/sensor_client.hpp"

namespace carla_telemetry {

class CarlaROS2Backend;

/**
 * @brief Manages camera and LiDAR sensor lifecycles.
 */
class CarlaSensorManager {
 public:
  CarlaSensorManager() = default;

  /**
   * @brief Add a camera.
   * @param cam Pointer to the camera.
   */
  void add_camera(std::unique_ptr<CarlaCamera> cam);
  /**
   * @brief Add a LiDAR.
   * @param lidar Pointer to the LiDAR.
   */
  void add_lidar(std::unique_ptr<CarlaLidar> lidar);
  /**
   * @brief Add a depth LiDAR.
   * @param depth_lidar Pointer to the depth LiDAR.
   */
  void add_depth_lidar(std::unique_ptr<CarlaDepthLidar> depth_lidar);
  /**
   * @brief Add a camera client.
   * @param cam_client Pointer to the camera client.
   */
  void add_camera_client(std::unique_ptr<CameraClient> cam_client);
  /**
   * @brief Add a LiDAR client.
   * @param lidar_client Pointer to the LiDAR client.
   */
  void add_lidar_client(std::unique_ptr<LidarClient> lidar_client);
  /**
   * @brief Add a depth LiDAR client.
   * @param depth_lidar_client Pointer to the depth LiDAR client.
   */
  void add_depth_lidar_client(
      std::unique_ptr<DepthLidarClient> depth_lidar_client);

  /**
   * @brief Start all sensors.
   * @param backend Pointer to the ROS 2 backend.
   */
  void start_all(CarlaROS2Backend* backend);
  /**
   * @brief Stop all sensors.
   */
  void stop_all();
  /**
   * @brief Destroy all sensors.
   */
  void destroy_all();

  /**
   * @brief Get the cameras.
   * @return Vector of cameras.
   */
  const std::vector<std::unique_ptr<CarlaCamera>>& cameras() const {
    return cameras_;
  }
  /**
   * @brief Get the LiDARs.
   * @return Vector of LiDARs.
   */
  const std::vector<std::unique_ptr<CarlaLidar>>& lidars() const {
    return lidars_;
  }

 private:
  // ── Sensor containers ──────────────────────────────────────────────
  std::vector<std::unique_ptr<CarlaCamera>> cameras_;
  std::vector<std::unique_ptr<CarlaLidar>> lidars_;
  std::vector<std::unique_ptr<CarlaDepthLidar>> depth_lidars_;
  std::vector<std::unique_ptr<CameraClient>> camera_clients_;
  std::vector<std::unique_ptr<LidarClient>> lidar_clients_;
  std::vector<std::unique_ptr<DepthLidarClient>> depth_lidar_clients_;
};

}  // namespace carla_telemetry
