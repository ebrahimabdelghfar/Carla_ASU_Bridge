#include "carla_telemetry/sensor_manager.hpp"

#include <iostream>

#include "carla_telemetry/ros2_backend.hpp"

namespace carla_telemetry {

void CarlaSensorManager::add_camera(std::unique_ptr<CarlaCamera> cam) {
  cameras_.push_back(std::move(cam));
}

void CarlaSensorManager::add_lidar(std::unique_ptr<CarlaLidar> lidar) {
  lidars_.push_back(std::move(lidar));
}

void CarlaSensorManager::add_depth_lidar(
    std::unique_ptr<CarlaDepthLidar> depth_lidar) {
  depth_lidars_.push_back(std::move(depth_lidar));
}

void CarlaSensorManager::add_camera_client(
    std::unique_ptr<CameraClient> cam_client) {
  camera_clients_.push_back(std::move(cam_client));
}

void CarlaSensorManager::add_lidar_client(
    std::unique_ptr<LidarClient> lidar_client) {
  lidar_clients_.push_back(std::move(lidar_client));
}

void CarlaSensorManager::add_depth_lidar_client(
    std::unique_ptr<DepthLidarClient> depth_lidar_client) {
  depth_lidar_clients_.push_back(std::move(depth_lidar_client));
}

void CarlaSensorManager::start_all(CarlaROS2Backend* backend) {
  for (auto& cam : cameras_) {
    cam->start(backend);
    std::cerr << "[SensorManager] Camera '" << cam->name() << "' started."
              << std::endl;
  }
  for (auto& lidar : lidars_) {
    lidar->start(backend);
    std::cerr << "[SensorManager] LiDAR '" << lidar->name() << "' started."
              << std::endl;
  }
  for (auto& dl : depth_lidars_) {
    dl->start(backend);
    std::cerr << "[SensorManager] DepthLiDAR '" << dl->name() << "' started."
              << std::endl;
  }
  for (auto& cc : camera_clients_) {
    cc->start(backend);
    std::cerr << "[SensorManager] CameraClient '" << cc->name() << "' started."
              << std::endl;
  }
  for (auto& lc : lidar_clients_) {
    lc->start(backend);
    std::cerr << "[SensorManager] LidarClient '" << lc->name() << "' started."
              << std::endl;
  }
  for (auto& dlc : depth_lidar_clients_) {
    dlc->start(backend);
    std::cerr << "[SensorManager] DepthLidarClient '" << dlc->name()
              << "' started." << std::endl;
  }
}

void CarlaSensorManager::stop_all() {
  for (auto& cam : cameras_) cam->stop();
  for (auto& lidar : lidars_) lidar->stop();
  for (auto& dl : depth_lidars_) dl->stop();
  for (auto& cc : camera_clients_) cc->stop();
  for (auto& lc : lidar_clients_) lc->stop();
  for (auto& dlc : depth_lidar_clients_) dlc->stop();
}

void CarlaSensorManager::destroy_all() {
  stop_all();
  for (auto& cam : cameras_) cam->destroy();
  for (auto& lidar : lidars_) lidar->destroy();
  for (auto& dl : depth_lidars_) dl->destroy();
  for (auto& cc : camera_clients_) cc->destroy();
  for (auto& lc : lidar_clients_) lc->destroy();
  for (auto& dlc : depth_lidar_clients_) dlc->destroy();
  cameras_.clear();
  lidars_.clear();
  depth_lidars_.clear();
  camera_clients_.clear();
  lidar_clients_.clear();
  depth_lidar_clients_.clear();
}

}  // namespace carla_telemetry
