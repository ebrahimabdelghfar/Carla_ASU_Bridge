#include "carla_telemetry/sensors/camera.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/geom/Transform.h>

#include <cmath>
#include <cstring>

#include "carla_telemetry/perf_monitor.hpp"
#include "carla_telemetry/ros2_backend.hpp"
#include "carla_telemetry/sensors/sensor_clock.hpp"

namespace carla_telemetry {

CarlaCamera::CarlaCamera(carla::client::World& world,
                         carla::SharedPtr<carla::client::Actor> vehicle,
                         const Config& cfg)
    : name_(cfg.name),
      frame_id_(cfg.frame_id.empty() ? cfg.name : cfg.frame_id),
      update_rate_(cfg.update_rate),
      update_period_(1.0 / static_cast<double>(cfg.update_rate)),
      image_size_x_(cfg.image_size_x),
      image_size_y_(cfg.image_size_y),
      fov_(cfg.fov) {
  compute_intrinsics();

  auto bp_lib = world.GetBlueprintLibrary();
  auto bp = *bp_lib->Find(cfg.type);
  bp.SetAttribute("image_size_x", std::to_string(image_size_x_));
  bp.SetAttribute("image_size_y", std::to_string(image_size_y_));
  if (bp.ContainsAttribute("fov")) {
    bp.SetAttribute("fov", std::to_string(fov_));
  }
  // No sensor_tick: don't throttle frame production. Receive frames as fast as
  // the server renders them; the publish thread is the only rate limiter.

  carla::geom::Transform transform(
      carla::geom::Location(cfg.sp_x, cfg.sp_y, cfg.sp_z),
      carla::geom::Rotation(cfg.sp_pitch, cfg.sp_yaw, cfg.sp_roll));

  auto actor = world.SpawnActor(bp, transform, vehicle.get());
  sensor_ = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
}

void CarlaCamera::compute_intrinsics() {
  double fx = static_cast<double>(image_size_x_) /
              (2.0 * std::tan(static_cast<double>(fov_) * M_PI / 360.0));
  double fy = fx;
  double cx = static_cast<double>(image_size_x_) / 2.0;
  double cy = static_cast<double>(image_size_y_) / 2.0;
  intrinsics_ = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
}

void CarlaCamera::on_image(carla::SharedPtr<carla::sensor::SensorData> data) {
  auto t_cb = PerfMonitor::tick();
  auto image = boost::dynamic_pointer_cast<carla::sensor::data::Image>(data);
  if (!image || !running_.load()) return;

  // Rate the server delivers frames to this client (inter-arrival interval).
  perf.record_interval(name_ + ".server_dt");

  // PTP-style shared clock: stamp with the CARLA simulation frame time mapped
  // through the process-wide sim->epoch anchor (see sensor_clock.hpp). Every
  // sensor ticked in the same world frame carries the identical sim timestamp,
  // so all camera + LiDAR header stamps for a frame agree well under 10 ms.
  double capture_time = sensor_sim_to_epoch(image->GetTimestamp());
  uint64_t frame = image->GetFrame();

  // Feed the shared server-rate estimator (drives the adaptive decimation).
  ServerRate::instance().report(frame);

  // Frames arrive at the full server render rate (no sensor_tick). Enqueue the
  // newest; the publish thread throttles by frame decimation. No gate here.
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (frame_queue_.size() >= MAX_QUEUE_SIZE) {
      frame_queue_.pop();
    }
    frame_queue_.push(FrameEntry{image, capture_time, t_cb, frame});
  }
  queue_cv_.notify_one();
  perf.record("cam.callback", t_cb);
}

void CarlaCamera::start(CarlaROS2Backend* backend) {
  if (running_.load()) return;
  ros2_backend_ = backend;
  running_.store(true);

  sensor_->Listen([this](auto data) { this->on_image(data); });
  publish_thread_ = std::thread(&CarlaCamera::publish_loop, this);
}

void CarlaCamera::stop() {
  running_.store(false);
  queue_cv_.notify_all();  // wake publish_loop out of its wait
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void CarlaCamera::destroy() {
  stop();
  if (sensor_) {
    try {
      sensor_->Stop();
      sensor_->Destroy();
    } catch (...) {
    }
    sensor_ = nullptr;
  }
}

void CarlaCamera::publish_loop() {
  while (running_.load()) {
    // Block until a frame arrives (or shutdown) — no fixed-period poll.
    FrameEntry entry{};
    bool have_frame = false;
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait(
          lk, [this] { return !frame_queue_.empty() || !running_.load(); });
      if (!running_.load()) break;
      // Shared rational resampling (the ONLY throttle): scan oldest→newest and
      // emit the first frame the predicate selects. sync_should_publish is a
      // deterministic function of the shared world frame number and the
      // measured server rate, so camera + LiDAR always emit the SAME world
      // frames (0 ms shift) at an average of exactly update_rate. Non-selected
      // frames are dropped; newer selected frames stay queued for the next
      // wake.
      while (!frame_queue_.empty()) {
        FrameEntry e = std::move(frame_queue_.front());
        frame_queue_.pop();
        if (sync_should_publish(e.frame, update_rate_)) {
          entry = std::move(e);
          have_frame = true;
          break;
        }
      }
    }

    if (!have_frame) continue;

    perf.record_value("cam.queue_depth", 0);  // drained

    try {
      auto t_parse = PerfMonitor::tick();
      auto cam_data = parse_image(entry.image, entry.capture_time);
      perf.record("cam.parse", t_parse);

      if (cam_data && ros2_backend_) {
        auto t_pub = PerfMonitor::tick();
        ros2_backend_->publish_camera_image(*cam_data);
        ros2_backend_->publish_camera_info(*cam_data);
        perf.record("cam.ros2_publish", t_pub);
        // End-to-end per this sensor/client: server callback → published.
        perf.record(name_ + ".total", entry.arrival_tick);
      }
    } catch (const std::exception& e) {
      // Log warning
    }
  }
}

std::optional<CameraData> CarlaCamera::parse_image(
    const carla::SharedPtr<carla::sensor::data::Image>& image,
    double capture_time) {
  if (!image || image->size() == 0) return std::nullopt;

  int width = static_cast<int>(image->GetWidth());
  int height = static_cast<int>(image->GetHeight());

  // CARLA images are BGRA (4 channels, uint8). Publish the native BGRA buffer
  // directly (encoding "bgra8") — a single bulk copy, no per-pixel channel
  // swizzle. Lossless: same pixel data, just channel order. Consumers read
  // bgra8 (rviz / image_transport / cv_bridge handle it natively).
  const auto* raw = reinterpret_cast<const uint8_t*>(image->data());
  size_t n_bytes = static_cast<size_t>(width) * static_cast<size_t>(height) * 4;

  std::vector<uint8_t> bgra(n_bytes);
  std::memcpy(bgra.data(), raw, n_bytes);

  CameraData d;
  d.rgb = std::move(bgra);
  d.intrinsics = intrinsics_;
  d.width = width;
  d.height = height;
  d.frame_id = frame_id_;
  d.encoding = "bgra8";
  d.capture_time = capture_time;
  return d;
}

}  // namespace carla_telemetry
