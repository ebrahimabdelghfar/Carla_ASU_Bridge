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

// Manages camera and LiDAR sensor lifecycles.
class CarlaSensorManager {
 public:
  CarlaSensorManager() = default;

  void add_camera(std::unique_ptr<CarlaCamera> cam);
  void add_lidar(std::unique_ptr<CarlaLidar> lidar);
  void add_depth_lidar(std::unique_ptr<CarlaDepthLidar> depth_lidar);
  void add_camera_client(std::unique_ptr<CameraClient> cam_client);
  void add_lidar_client(std::unique_ptr<LidarClient> lidar_client);
  void add_depth_lidar_client(
      std::unique_ptr<DepthLidarClient> depth_lidar_client);

  void start_all(CarlaROS2Backend* backend);
  void stop_all();
  void destroy_all();

  const std::vector<std::unique_ptr<CarlaCamera>>& cameras() const {
    return cameras_;
  }
  const std::vector<std::unique_ptr<CarlaLidar>>& lidars() const {
    return lidars_;
  }

 private:
  std::vector<std::unique_ptr<CarlaCamera>> cameras_;
  std::vector<std::unique_ptr<CarlaLidar>> lidars_;
  std::vector<std::unique_ptr<CarlaDepthLidar>> depth_lidars_;
  std::vector<std::unique_ptr<CameraClient>> camera_clients_;
  std::vector<std::unique_ptr<LidarClient>> lidar_clients_;
  std::vector<std::unique_ptr<DepthLidarClient>> depth_lidar_clients_;
};

}  // namespace carla_telemetry
