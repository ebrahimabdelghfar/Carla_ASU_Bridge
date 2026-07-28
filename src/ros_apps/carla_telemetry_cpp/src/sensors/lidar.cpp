#include "carla_telemetry/sensors/lidar.hpp"

#include <carla/actors/BlueprintLibrary.h>
#include <carla/geom/Transform.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/SemanticLidarMeasurement.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include "carla_telemetry/perf_monitor.hpp"
#include "carla_telemetry/ros2_backend.hpp"
#include "carla_telemetry/sensors/sensor_clock.hpp"

namespace carla_telemetry {

namespace {
constexpr int ROTARY_FIELDS = 4;  // x, y, z, intensity
constexpr int SOLID_FIELDS = 6;  // x, y, z, cos_angle, object_idx, semantic_tag
}  // namespace

CarlaLidar::CarlaLidar(carla::client::World& world,
                       carla::SharedPtr<carla::client::Actor> vehicle,
                       const Config& cfg)
    : name_(cfg.name),
      frame_id_(cfg.frame_id.empty() ? cfg.name : cfg.frame_id),
      lidar_type_(cfg.lidar_type),
      topic_point_cloud_(cfg.topic_point_cloud.empty()
                             ? "lidar/" + cfg.name + "/points"
                             : cfg.topic_point_cloud),
      update_rate_(cfg.update_rate),
      update_period_(1.0 / static_cast<double>(cfg.update_rate)),
      is_solid_state_(cfg.lidar_type == "solid_state"),
      is_gpu_(cfg.lidar_type == "gpu"),
      n_fields_(is_solid_state_ ? SOLID_FIELDS : ROTARY_FIELDS),
      rotation_freq_(cfg.rotation_frequency) {
  auto bp_lib = world.GetBlueprintLibrary();
  std::string blueprint_id;
  if (is_solid_state_)
    blueprint_id = "sensor.lidar.ray_cast_semantic";
  else if (is_gpu_)
    blueprint_id = "sensor.lidar.ray_cast_gpu";  // custom GPU sensor
  else
    blueprint_id = "sensor.lidar.ray_cast";

  auto bp = *bp_lib->Find(blueprint_id);

  auto set_attr = [&](const std::string& key, const std::string& val) {
    if (bp.ContainsAttribute(key)) bp.SetAttribute(key, val);
  };

  set_attr("channels", std::to_string(cfg.channels));
  set_attr("range", std::to_string(cfg.range));
  set_attr("points_per_second", std::to_string(cfg.points_per_second));

  if (is_solid_state_) {
    set_attr("horizontal_fov", std::to_string(cfg.horizontal_fov));
    set_attr("vertical_fov", std::to_string(cfg.vertical_fov));
  } else {
    set_attr("rotation_frequency", std::to_string(cfg.rotation_frequency));
    set_attr("upper_fov", std::to_string(cfg.upper_fov));
    set_attr("lower_fov", std::to_string(cfg.lower_fov));
    set_attr("atmosphere_attenuation_rate",
             std::to_string(cfg.atmosphere_attenuation_rate));
    set_attr("dropoff_general_rate", std::to_string(cfg.dropoff_general_rate));
    set_attr("dropoff_intensity_limit",
             std::to_string(cfg.dropoff_intensity_limit));
    set_attr("dropoff_zero_intensity",
             std::to_string(cfg.dropoff_zero_intensity));
    // GPU-only attribute (guarded — ignored by ray_cast). Selects the
    // compute-shader sampling path in the custom ray_cast_gpu sensor.
    set_attr("use_compute", cfg.use_compute ? "true" : "false");
  }

  carla::geom::Transform transform(
      carla::geom::Location(cfg.sp_x, cfg.sp_y, cfg.sp_z),
      carla::geom::Rotation(cfg.sp_pitch, cfg.sp_yaw, cfg.sp_roll));

  auto actor = world.SpawnActor(bp, transform, vehicle.get());
  sensor_ = boost::dynamic_pointer_cast<carla::client::Sensor>(actor);
}

void CarlaLidar::on_lidar(carla::SharedPtr<carla::sensor::SensorData> data) {
  auto t_cb = PerfMonitor::tick();
  if (!running_.load()) {
    perf.record("lidar.callback.early_exit", t_cb);
    return;
  }

  // Rate the server delivers measurements to this client (inter-arrival).
  perf.record_interval(name_ + ".server_dt");

  // PTP-style shared clock (see sensor_clock.hpp): map the CARLA sim frame time
  // through the same process-wide anchor the cameras use, so LiDAR + camera
  // header stamps for one world frame match within a world tick.
  double sim_ts = data->GetTimestamp();
  double capture_time = sensor_sim_to_epoch(sim_ts);
  uint64_t frame = data->GetFrame();

  // Feed the shared server-rate estimator (drives the adaptive decimation).
  ServerRate::instance().report(frame);

  std::vector<AccumChunk> ready_chunks;
  double publish_capture_time = capture_time;
  double publish_arrival_tick = t_cb;
  uint64_t publish_frame = frame;

  {
    std::lock_guard<std::mutex> lk(accum_lock_);
    if (!is_solid_state_) {
      // Rotary: accumulate sub-sweep measurements into one revolution.
      double ts = sim_ts;
      double rotation_duration =
          1.0 / std::max(static_cast<double>(rotation_freq_), 1.0);

      // Sim-time gap since the previous callback. When the server already
      // delivers >= one revolution per callback (rotation_frequency >= sim
      // fps), frame_dt >= rotation period -> publish each callback standalone.
      // Only stitch when callbacks are thin sub-sweeps (fps > rotation_freq).
      // Without this the timestamp-span gate always defers the first chunk
      // (span 0), forcing >=2 callbacks per publish and halving the rate.
      double frame_dt = (last_cb_ts_ > 0.0) ? (ts - last_cb_ts_) : 0.0;
      last_cb_ts_ = ts;

      if (accum_start_time_ < 0.0) {
        accum_start_time_ = ts;
        accum_capture_time_ = capture_time;
        accum_arrival_tick_ = t_cb;
      }
      accum_data_.push_back(AccumChunk{data});

      bool full_rev_per_frame = frame_dt >= (rotation_duration * 0.9);
      if (!full_rev_per_frame &&
          (ts - accum_start_time_) < (rotation_duration * 0.9)) {
        perf.record("lidar.callback.accumulate", t_cb);
        return;  // keep accumulating
      }

      ready_chunks = std::move(accum_data_);
      publish_capture_time = accum_capture_time_;
      publish_arrival_tick = accum_arrival_tick_;
      accum_data_.clear();
      accum_start_time_ = -1.0;
    } else {
      ready_chunks.push_back(AccumChunk{data});
    }
  }

  // Push to queue
  {
    auto t_queue = PerfMonitor::tick();
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (frame_queue_.size() >= MAX_QUEUE_SIZE) {
      frame_queue_.pop();
    }
    frame_queue_.push(QueueEntry{std::move(ready_chunks), publish_capture_time,
                                 publish_arrival_tick, publish_frame});
    perf.record("lidar.callback.queue_push", t_queue);
  }
  queue_cv_.notify_one();
  perf.record("lidar.callback.publish_ready", t_cb);
}

void CarlaLidar::parse_into(
    const carla::SharedPtr<carla::sensor::SensorData>& data,
    std::vector<uint8_t>& out) {
  auto t_parse_func = PerfMonitor::tick();

  if (!is_solid_state_) {
    auto lidar =
        boost::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(
            data);
    if (!lidar || lidar->size() == 0) {
      perf.record("lidar.parse_lidar.cast_fail", t_parse_func);
      return;
    }
    // raw_data is contiguous LidarDetection{x,y,z,intensity} (16B) — bulk copy,
    // then negate Y in place (CARLA→ROS). Matches native CarlaLidarPublisher.
    const size_t n = lidar->size();
    const size_t bytes = n * sizeof(carla::sensor::data::LidarDetection);
    const size_t off = out.size();
    out.resize(off + bytes);
    std::memcpy(out.data() + off, lidar->begin(), bytes);
    auto* det = reinterpret_cast<carla::sensor::data::LidarDetection*>(
        out.data() + off);
    for (size_t i = 0; i < n; ++i) det[i].point.y = -det[i].point.y;
  } else {
    auto sem = boost::dynamic_pointer_cast<
        carla::sensor::data::SemanticLidarMeasurement>(data);
    if (!sem) {
      perf.record("lidar.parse_lidar.cast_fail", t_parse_func);
      return;
    }
    // Semantic detection stride differs (24B) — de-interleave XYZ into XYZI
    // (intensity 0).
    const size_t n = sem->size();
    const size_t off = out.size();
    out.resize(off + n * 16);
    float* f = reinterpret_cast<float*>(out.data() + off);
    size_t idx = 0;
    for (const auto& det : *sem) {
      f[idx * 4 + 0] = det.point.x;
      f[idx * 4 + 1] = -det.point.y;  // Y flip (CARLA→ROS)
      f[idx * 4 + 2] = det.point.z;
      f[idx * 4 + 3] = 0.0f;  // no intensity on semantic
      ++idx;
    }
  }

  perf.record("lidar.parse_lidar.total", t_parse_func);
}

void CarlaLidar::start(CarlaROS2Backend* backend) {
  if (running_.load()) return;
  ros2_backend_ = backend;
  running_.store(true);

  sensor_->Listen([this](auto data) { this->on_lidar(data); });
  publish_thread_ = std::thread(&CarlaLidar::publish_loop, this);
}

void CarlaLidar::stop() {
  running_.store(false);
  queue_cv_.notify_all();  // wake publish_loop out of its wait
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void CarlaLidar::destroy() {
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

void CarlaLidar::publish_loop() {
  while (running_.load()) {
    // Block until a completed sweep arrives (or shutdown) — no fixed poll.
    QueueEntry qe;
    bool have_entry = false;
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      // Block (idle, no CPU) until a sweep arrives or shutdown.
      queue_cv_.wait(
          lk, [this] { return !frame_queue_.empty() || !running_.load(); });
      if (!running_.load()) break;
      // Timer starts AFTER the wait — measure drain work, not idle wait.
      auto t_queue_pop = PerfMonitor::tick();
      // Shared rational resampling (the ONLY throttle): scan oldest→newest and
      // emit the first sweep the predicate selects. sync_should_publish is a
      // deterministic function of the shared world frame number and the
      // measured server rate, so LiDAR + cameras emit the SAME world frames → 0
      // ms shift at an average of exactly update_rate. Server requests /
      // accumulation are untouched — this gates the ROS publish only.
      while (!frame_queue_.empty()) {
        QueueEntry e = std::move(frame_queue_.front());
        frame_queue_.pop();
        if (sync_should_publish(e.frame, update_rate_)) {
          qe = std::move(e);
          have_entry = true;
          break;
        }
      }
      perf.record("lidar.publish_loop.queue_pop", t_queue_pop);
    }
    if (!have_entry) continue;

    perf.record_value("lidar.queue_depth", 0);

    // Parse all chunks straight into one interleaved XYZI byte buffer.
    std::vector<uint8_t> full_data;

    auto t_parse = PerfMonitor::tick();
    for (auto& chunk : qe.chunks) {
      parse_into(chunk.data, full_data);
    }
    perf.record("lidar.publish_loop.parse_total", t_parse);

    if (full_data.empty()) continue;

    try {
      if (ros2_backend_) {
        auto t_pub = PerfMonitor::tick();

        LidarData ld;
        ld.num_points = full_data.size() / 16;
        ld.data = std::move(full_data);
        ld.frame_id = frame_id_;
        ld.lidar_name = name_;
        ld.capture_time = qe.capture_time;

        ros2_backend_->publish_point_cloud(ld);
        perf.record("lidar.ros2_publish", t_pub);
        // End-to-end per this sensor/client: server callback → published.
        perf.record(name_ + ".total", qe.arrival_tick);
      }
    } catch (...) {
    }
  }
}

}  // namespace carla_telemetry
