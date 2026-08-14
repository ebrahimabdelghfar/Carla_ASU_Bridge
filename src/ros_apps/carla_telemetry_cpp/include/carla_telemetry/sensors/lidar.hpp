#pragma once

#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/SensorData.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

class CarlaROS2Backend;  // forward

// CARLA LiDAR sensor wrapper — supports rotary and solid-state.
// Direct port of CarlaLidar from lidar.py.
class CarlaLidar {
 public:
  struct Config {
    std::string name = "lidar";
    std::string frame_id = "lidar";
    std::string lidar_type = "rotary";  // "rotary" | "solid_state" | "gpu"
    bool use_compute = false;           // gpu only: GPU compute-shader path
    float update_rate = 20.0f;
    // Spawn point
    float sp_x = 0.0f, sp_y = 0.0f, sp_z = 2.4f;
    float sp_roll = 0.0f, sp_pitch = 0.0f, sp_yaw = 0.0f;
    // Common
    int channels = 64;
    float range = 85.0f;
    int points_per_second = 600000;
    // Rotary
    float rotation_frequency = 20.0f;
    float upper_fov = 10.0f;
    float lower_fov = -30.0f;
    float atmosphere_attenuation_rate = 0.004f;
    float dropoff_general_rate = 0.45f;
    float dropoff_intensity_limit = 0.8f;
    float dropoff_zero_intensity = 0.4f;
    // Solid-state
    float horizontal_fov = 120.0f;
    float vertical_fov = 30.0f;

    std::string topic_point_cloud;
  };

  CarlaLidar(carla::client::World& world,
             carla::SharedPtr<carla::client::Actor> vehicle, const Config& cfg);

  void start(CarlaROS2Backend* backend);
  void stop();
  void destroy();

  const std::string& name() const { return name_; }
  const std::string& frame_id() const { return frame_id_; }
  const std::string& lidar_type() const { return lidar_type_; }
  const std::string& topic_point_cloud() const { return topic_point_cloud_; }

 private:
  void on_lidar(carla::SharedPtr<carla::sensor::SensorData> data);

  // Parse the lidar data into interleaved XYZI float32 (16 bytes/point).
  void parse_into(const carla::SharedPtr<carla::sensor::SensorData>& data,
                  std::vector<uint8_t>& out);

  void publish_loop();

  std::string name_;
  std::string frame_id_;
  std::string lidar_type_;
  std::string topic_point_cloud_;
  float update_rate_;
  double update_period_;
  bool is_solid_state_;
  bool is_gpu_;
  int n_fields_;
  float rotation_freq_;

  // Sweep accumulation
  std::mutex accum_lock_;
  struct AccumChunk {
    carla::SharedPtr<carla::sensor::SensorData> data;
  };
  std::vector<AccumChunk> accum_data_;
  double accum_start_time_ = -1.0;
  double accum_capture_time_ = 0.0;
  double accum_arrival_tick_ =
      0.0;                    // PerfMonitor::tick() at sweep's first chunk
  double last_cb_ts_ = -1.0;  // prev callback sim timestamp (frame-dt detect)
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  struct QueueEntry {
    std::vector<AccumChunk> chunks;
    double capture_time;
    double arrival_tick;  // for end-to-end total (server callback → published)
    uint64_t frame;  // CARLA world frame number — shared across all sensors
  };
  std::queue<QueueEntry> frame_queue_;
  static constexpr size_t MAX_QUEUE_SIZE = 20;

  CarlaROS2Backend* ros2_backend_ = nullptr;
  std::atomic<bool> running_{false};
  std::thread publish_thread_;
  carla::SharedPtr<carla::client::Sensor> sensor_;
};

}  // namespace carla_telemetry
