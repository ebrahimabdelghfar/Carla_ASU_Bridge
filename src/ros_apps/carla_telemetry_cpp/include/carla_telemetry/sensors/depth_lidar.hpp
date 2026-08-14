#pragma once

#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/data/Image.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

class CarlaROS2Backend;  // forward

// GPU depth-camera LiDAR: a ring of `sensor.camera.depth` cameras (rendered on
// the GPU, no CPU raycasting) whose depth pixels are reprojected to a single
// PointCloud2 in the LiDAR frame. Real-time, async, does not touch the RGB
// cameras. Different sensor model than ray_cast — a reprojected depth raster,
// not a spinning-beam scan: no intensity, no per-beam dropoff/atmosphere.
//
// Satisfies the SensorClient TSensor contract (nested Config with `name`,
// ctor(World&, SharedPtr<Actor>, const Config&), start/stop/destroy/name).
class CarlaDepthLidar {
 public:
  struct Config {
    std::string name = "depth_lidar";
    std::string frame_id = "lidar";
    std::string topic_point_cloud;
    float update_rate = 10.0f;
    // Mount (shared by every camera in the ring)
    float sp_x = 0.0f, sp_y = 0.0f, sp_z = 2.4f;
    float sp_roll = 0.0f, sp_pitch = 0.0f, sp_yaw = 0.0f;
    // Ring / camera parameters
    int num_cameras = 6;     // depth cameras around the ring
    int image_size_x = 400;  // per-camera width
    int image_size_y = 300;  // per-camera height
    float fov = 0.0f;        // per-camera horizontal FOV (deg); <=0 → auto
    float range = 100.0f;    // max depth kept (m)
    float min_range = 0.5f;  // min depth kept (m)
    int point_stride = 2;    // pixel decimation (1 = every pixel)
  };

  CarlaDepthLidar(carla::client::World& world,
                  carla::SharedPtr<carla::client::Actor> vehicle,
                  const Config& cfg);

  void start(CarlaROS2Backend* backend);
  void stop();
  void destroy();

  const std::string& name() const { return name_; }
  const std::string& frame_id() const { return frame_id_; }

 private:
  void on_image(size_t cam_index,
                carla::SharedPtr<carla::sensor::SensorData> data);
  void publish_loop();
  // Precomputes, per camera, the reprojection ray (in the ROS LiDAR frame)
  // and the byte offset for every sampled pixel.
  void build_ray_table(int cam_index, float yaw_deg);

  std::string name_;
  std::string frame_id_;
  std::string topic_point_cloud_;
  float update_rate_;
  double update_period_;
  int width_, height_;
  float range_, min_range_;
  int stride_;
  int num_cameras_;
  float fov_deg_;

  // Per-camera precomputed reprojection: sampled pixel byte offsets + rays.
  struct RayTable {
    std::vector<size_t> offset;  // byte offset into BGRA buffer
    std::vector<float> ray;      // 3 floats per sample (ROS LiDAR frame)
  };
  std::vector<RayTable> ray_tables_;

  // Latest depth frame per camera.
  std::mutex frames_mutex_;
  std::vector<carla::SharedPtr<carla::sensor::data::Image>> latest_frames_;

  CarlaROS2Backend* ros2_backend_ = nullptr;
  std::atomic<bool> running_{false};
  std::thread publish_thread_;
  std::vector<carla::SharedPtr<carla::client::Sensor>> sensors_;
};

}  // namespace carla_telemetry
