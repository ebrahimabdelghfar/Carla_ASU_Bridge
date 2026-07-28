#include "carla_telemetry/sensors/depth_lidar.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/geom/Transform.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include "carla_telemetry/perf_monitor.hpp"
#include "carla_telemetry/ros2_backend.hpp"

namespace carla_telemetry {

CarlaDepthLidar::CarlaDepthLidar(carla::client::World& world,
                                 carla::SharedPtr<carla::client::Actor> vehicle,
                                 const Config& cfg)
    : name_(cfg.name),
      frame_id_(cfg.frame_id.empty() ? cfg.name : cfg.frame_id),
      topic_point_cloud_(cfg.topic_point_cloud.empty()
                             ? "lidar/" + cfg.name + "/points"
                             : cfg.topic_point_cloud),
      update_rate_(cfg.update_rate),
      update_period_(1.0 /
                     static_cast<double>(std::max(cfg.update_rate, 1.0f))),
      width_(cfg.image_size_x),
      height_(cfg.image_size_y),
      range_(cfg.range),
      min_range_(cfg.min_range),
      stride_(std::max(cfg.point_stride, 1)),
      num_cameras_(std::max(cfg.num_cameras, 1))
      // Auto-tile the ring with a small overlap so seams don't leave gaps.
      ,
      fov_deg_(cfg.fov > 0.0f ? cfg.fov
                              : (360.0f / static_cast<float>(
                                              std::max(cfg.num_cameras, 1))) +
                                    4.0f) {
  latest_frames_.resize(num_cameras_);
  ray_tables_.resize(num_cameras_);

  auto bp_lib = world.GetBlueprintLibrary();
  auto bp = *bp_lib->Find("sensor.camera.depth");
  bp.SetAttribute("image_size_x", std::to_string(width_));
  bp.SetAttribute("image_size_y", std::to_string(height_));
  if (bp.ContainsAttribute("fov"))
    bp.SetAttribute("fov", std::to_string(fov_deg_));
  if (bp.ContainsAttribute("sensor_tick")) {
    bp.SetAttribute("sensor_tick", std::to_string(update_period_));
  }

  for (int i = 0; i < num_cameras_; ++i) {
    float yaw_i =
        static_cast<float>(i) * (360.0f / static_cast<float>(num_cameras_));
    build_ray_table(i, yaw_i);

    // All cameras share the mount location; each adds its ring yaw.
    carla::geom::Transform transform(
        carla::geom::Location(cfg.sp_x, cfg.sp_y, cfg.sp_z),
        carla::geom::Rotation(cfg.sp_pitch, cfg.sp_yaw + yaw_i, cfg.sp_roll));
    auto actor = world.SpawnActor(bp, transform, vehicle.get());
    sensors_.push_back(
        boost::dynamic_pointer_cast<carla::client::Sensor>(actor));
  }
}

// Precompute, for one camera, the reprojection ray of every sampled pixel,
// expressed in the ROS LiDAR frame (x fwd, y left, z up). CARLA depth is planar
// (along the optical axis), so world_point = depth * ray.
void CarlaDepthLidar::build_ray_table(int cam_index, float yaw_deg) {
  const double fx =
      static_cast<double>(width_) /
      (2.0 * std::tan(static_cast<double>(fov_deg_) * M_PI / 360.0));
  const double fy = fx;  // square pixels
  const double cx = static_cast<double>(width_) / 2.0;
  const double cy = static_cast<double>(height_) / 2.0;
  const double yaw = static_cast<double>(yaw_deg) * M_PI / 180.0;
  const double cyaw = std::cos(yaw), syaw = std::sin(yaw);

  RayTable& rt = ray_tables_[cam_index];
  rt.offset.clear();
  rt.ray.clear();
  for (int v = 0; v < height_; v += stride_) {
    for (int u = 0; u < width_; u += stride_) {
      const double a = (static_cast<double>(u) - cx) / fx;
      const double b = (static_cast<double>(v) - cy) / fy;
      // optical(x right, y down, z fwd) ray (a,b,1) → CARLA body (fwd,right,up)
      const double bx = 1.0, by = a, bz = -b;
      // rotate by ring yaw about the mount's up axis (CARLA left-handed Z)
      const double rx = bx * cyaw - by * syaw;
      const double ry = bx * syaw + by * cyaw;
      const double rz = bz;
      // CARLA body → ROS (flip Y)
      rt.ray.push_back(static_cast<float>(rx));
      rt.ray.push_back(static_cast<float>(-ry));
      rt.ray.push_back(static_cast<float>(rz));
      rt.offset.push_back(static_cast<size_t>(v) * static_cast<size_t>(width_) *
                              4u +
                          static_cast<size_t>(u) * 4u);
    }
  }
}

void CarlaDepthLidar::on_image(
    size_t cam_index, carla::SharedPtr<carla::sensor::SensorData> data) {
  if (!running_.load()) return;
  auto image = boost::dynamic_pointer_cast<carla::sensor::data::Image>(data);
  if (!image) return;
  // Rate the server delivers each depth camera stream to this client.
  perf.record_interval(name_ + ".server_dt.cam" + std::to_string(cam_index));
  std::lock_guard<std::mutex> lk(frames_mutex_);
  latest_frames_[cam_index] = image;
}

void CarlaDepthLidar::start(CarlaROS2Backend* backend) {
  if (running_.load()) return;
  ros2_backend_ = backend;
  running_.store(true);
  for (size_t i = 0; i < sensors_.size(); ++i) {
    if (!sensors_[i]) continue;
    sensors_[i]->Listen([this, i](auto data) { this->on_image(i, data); });
  }
  publish_thread_ = std::thread(&CarlaDepthLidar::publish_loop, this);
}

void CarlaDepthLidar::stop() {
  running_.store(false);
  if (publish_thread_.joinable()) publish_thread_.join();
}

void CarlaDepthLidar::destroy() {
  stop();
  for (auto& s : sensors_) {
    if (s) {
      try {
        s->Stop();
        s->Destroy();
      } catch (...) {
      }
    }
  }
  sensors_.clear();
}

// Decode CARLA depth (BGRA -> normalized -> metres, 1000 m far plane).
static inline float decode_depth(const uint8_t* px) {
  const float r = px[2], g = px[1], b = px[0];
  const float normalized = (r + g * 256.0f + b * 65536.0f) / 16777215.0f;
  return normalized * 1000.0f;
}

void CarlaDepthLidar::publish_loop() {
  using clock = std::chrono::steady_clock;
  auto next = clock::now();

  while (running_.load()) {
    auto now = clock::now();
    if (next > now) std::this_thread::sleep_until(next);
    next += std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(update_period_));
    if (next < clock::now()) {
      next = clock::now() + std::chrono::duration_cast<clock::duration>(
                                std::chrono::duration<double>(update_period_));
    }

    // Snapshot the latest frame from each camera.
    std::vector<carla::SharedPtr<carla::sensor::data::Image>> frames;
    {
      std::lock_guard<std::mutex> lk(frames_mutex_);
      frames = latest_frames_;
    }

    auto t_reproj = PerfMonitor::tick();
    std::vector<float> points;  // N*4 flat XYZI (ROS LiDAR frame, intensity 0)
    for (size_t c = 0; c < frames.size(); ++c) {
      const auto& img = frames[c];
      if (!img || img->size() == 0) continue;
      const auto* raw = reinterpret_cast<const uint8_t*>(img->data());
      const RayTable& rt = ray_tables_[c];
      const size_t n = rt.offset.size();
      points.reserve(points.size() + n * 4);
      for (size_t k = 0; k < n; ++k) {
        const float d = decode_depth(raw + rt.offset[k]);
        if (d < min_range_ || d >= range_) continue;
        const float* ray = &rt.ray[k * 3];
        points.push_back(d * ray[0]);
        points.push_back(d * ray[1]);
        points.push_back(d * ray[2]);
        points.push_back(0.0f);
      }
    }
    perf.record("depth_lidar.reproject", t_reproj);

    if (points.empty() || !ros2_backend_) continue;

    double capture_time =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();

    try {
      auto t_pub = PerfMonitor::tick();
      LidarData ld;
      ld.num_points = points.size() / 4;
      ld.data.resize(points.size() * sizeof(float));
      std::memcpy(ld.data.data(), points.data(), ld.data.size());
      ld.frame_id = frame_id_;
      ld.lidar_name = name_;
      ld.capture_time = capture_time;
      ros2_backend_->publish_point_cloud(ld);  // XYZI interleaved, intensity 0
      perf.record("depth_lidar.ros2_publish", t_pub);
      // End-to-end per this sensor/client: reproject → published.
      perf.record(name_ + ".total", t_reproj);
    } catch (...) {
    }
  }
}

}  // namespace carla_telemetry
