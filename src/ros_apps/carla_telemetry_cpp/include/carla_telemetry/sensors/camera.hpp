#pragma once

#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/sensor/data/Image.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>

#include "carla_telemetry/types.hpp"

namespace carla_telemetry {

class CarlaROS2Backend;  // forward

/// CARLA camera sensor wrapper with per-camera publish thread.
/// Direct port of CarlaCamera from camera.py.
class CarlaCamera {
 public:
  struct Config {
    std::string name = "camera";
    std::string frame_id = "camera";
    std::string type = "sensor.camera.rgb";
    float update_rate = 30.0f;
    int image_size_x = 640;
    int image_size_y = 480;
    float fov = 90.0f;
    std::string topic_rgb;
    std::string topic_camera_info;
    // spawn point
    float sp_x = 1.5f, sp_y = 0.0f, sp_z = 1.5f;
    float sp_roll = 0.0f, sp_pitch = 0.0f, sp_yaw = 0.0f;
  };

  CarlaCamera(carla::client::World& world,
              carla::SharedPtr<carla::client::Actor> vehicle,
              const Config& cfg);

  void start(CarlaROS2Backend* backend);
  void stop();
  void destroy();

  const std::string& name() const { return name_; }
  const std::string& frame_id() const { return frame_id_; }

 private:
  struct FrameEntry {
    carla::SharedPtr<carla::sensor::data::Image> image;
    double capture_time;
    double
        arrival_tick;  // PerfMonitor::tick() at callback — for end-to-end total
    uint64_t frame;    // CARLA world frame number — shared across all sensors
  };

  void on_image(carla::SharedPtr<carla::sensor::SensorData> data);
  void publish_loop();
  std::optional<CameraData> parse_image(
      const carla::SharedPtr<carla::sensor::data::Image>& image,
      double capture_time);
  void compute_intrinsics();

  std::string name_;
  std::string frame_id_;
  float update_rate_;
  double update_period_;
  int image_size_x_, image_size_y_;
  float fov_;

  std::array<double, 9> intrinsics_{};

  // Thread-safe bounded queue. publish_loop is event-driven: it blocks on
  // queue_cv_ until a frame arrives. Frames are produced at the full server
  // render rate (no sensor_tick); the publish thread is the only rate limiter.
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<FrameEntry> frame_queue_;
  static constexpr size_t MAX_QUEUE_SIZE = 20;

  // Publisher-side rate control via shared rational resampling: publish a frame
  // iff sync_should_publish(frame, update_rate) (see sensor_clock.hpp).
  // Stateless deterministic function of the shared world frame number and the
  // measured server rate, so every sensor emits the SAME frames (0 ms shift) at
  // an average of exactly update_rate — even when the server can't hold its
  // configured tick rate, and with no stall when the rate estimate changes.

  CarlaROS2Backend* ros2_backend_ = nullptr;
  std::atomic<bool> running_{false};
  std::thread publish_thread_;
  carla::SharedPtr<carla::client::Sensor> sensor_;
};

}  // namespace carla_telemetry
