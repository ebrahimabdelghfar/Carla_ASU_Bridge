#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>

namespace carla_telemetry {

// PTP-style "grandmaster" clock shared by every sensor and stamper.
class SimClock {
 public:
  static SimClock& instance() {
    static SimClock c;
    return c;
  }

  // Convert a sensor frame's sim timestamp to the shared epoch clock.
  double to_epoch(double sim_ts) {
    std::lock_guard<std::mutex> lk(m_);
    double wall = wall_now();
    if (!init_) {
      sim0_ = sim_ts;
      epoch0_ = wall;
      init_ = true;
    }
    if (sim_ts >= latest_sim_) {
      // Estimate sim-per-wall rate from the gap to the previous frame.
      // The server runs slower than realtime (e.g. 14 fps @ 0.05s step =
      // 0.7s sim per wall second), so rate < 1. EMA-smoothed; ignore
      // degenerate deltas.
      double dw = wall - latest_wall_;
      double ds = sim_ts - latest_sim_;
      if (have_rate_ && dw > 1e-4 && ds > 0.0) {
        double inst = ds / dw;
        if (inst > 0.0 && inst < 2.0) rate_ = 0.9 * rate_ + 0.1 * inst;
      } else if (dw > 1e-4 && ds > 0.0) {
        rate_ = ds / dw;
        have_rate_ = true;
      }
      latest_sim_ = sim_ts;
      latest_wall_ = wall;
    }
    return epoch0_ + (sim_ts - sim0_);
  }

  // Get the latest sim-epoch, without wall extrapolation.
  double latest_epoch() {
    std::lock_guard<std::mutex> lk(m_);
    if (!init_) return wall_now();  // no frame yet: fall back
    return epoch0_ + (latest_sim_ - sim0_);
  }

  // Get the current sim-epoch, extrapolated by real elapsed wall time.
  double now_epoch() {
    std::lock_guard<std::mutex> lk(m_);
    if (!init_) return wall_now();  // no frame yet: fall back
    double extra = (wall_now() - latest_wall_) * (have_rate_ ? rate_ : 0.0);
    if (extra < 0.0) extra = 0.0;
    if (extra > MAX_EXTRAP)
      extra = MAX_EXTRAP;  // don't run away if sensors stall
    double cand = epoch0_ + (latest_sim_ - sim0_) + extra;
    // Strictly increasing, never merely flat: every real frame arrival
    // re-anchors latest_sim_/latest_wall_, so the next candidate dips below the
    // previous return value. Clamping to last_now_ would emit identical stamps
    // for several ticks until wall time caught up; bump by one ULP instead.
    if (cand <= last_now_)
      cand = std::nextafter(last_now_, std::numeric_limits<double>::infinity());
    last_now_ = cand;
    return cand;
  }

 private:
  static double wall_now() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }
  static constexpr double MAX_EXTRAP = 0.2;  // s

  std::mutex m_;
  bool init_ = false;
  double sim0_ = 0.0;
  double epoch0_ = 0.0;
  double latest_sim_ = 0.0;
  double latest_wall_ = 0.0;
  double rate_ = 0.0;  // measured sim seconds per wall second (server pace)
  bool have_rate_ = false;
  double last_now_ = 0.0;  // monotonic guard for now_epoch()
};

inline double sensor_sim_to_epoch(double sim_ts) {
  return SimClock::instance().to_epoch(sim_ts);
}

inline double sim_now_epoch() { return SimClock::instance().now_epoch(); }

inline double sim_latest_epoch() { return SimClock::instance().latest_epoch(); }

// Process-wide estimator of the server's actual wall-clock frame rate, shared
// by all sensors. The publish decimation is derived from this instead of the
// config target, so when the server can't hold fixed_delta (e.g. runs 14 Hz
// not 20 Hz) every sensor still emits its frames at >= its update_rate rather
// than collapsing.
class ServerRate {
 public:
  static ServerRate& instance() {
    static ServerRate r;
    return r;
  }

  // Feed the newest world frame seen by any sensor. Recomputes fps over a ~1 s
  // sliding wall window. Cheap; safe to call from every sensor callback.
  void report(uint64_t frame) {
    std::lock_guard<std::mutex> lk(m_);
    auto now = std::chrono::steady_clock::now();
    if (!started_) {
      win_start_ = now;
      win_start_frame_ = frame;
      win_frame_ = frame;
      started_ = true;
      return;
    }
    if (frame > win_frame_) win_frame_ = frame;
    double dt = std::chrono::duration<double>(now - win_start_).count();
    if (dt >= 1.0 && win_frame_ > win_start_frame_) {
      fps_.store(static_cast<double>(win_frame_ - win_start_frame_) / dt,
                 std::memory_order_relaxed);
      win_start_ = now;
      win_start_frame_ = win_frame_;
    }
  }

  double fps() const { return fps_.load(std::memory_order_relaxed); }

 private:
  std::mutex m_;
  std::chrono::steady_clock::time_point win_start_{};
  uint64_t win_start_frame_ = 0;
  uint64_t win_frame_ = 0;
  bool started_ = false;
  std::atomic<double> fps_{0.0};
};

// Check if the current frame should be published at the given update rate.
inline bool sync_should_publish(uint64_t frame, double update_rate) {
  double fps = ServerRate::instance().fps();
  if (fps <= 0.0 || update_rate <= 0.0 || frame == 0)
    return true;                        // warmup: every frame
  double sfps = std::floor(fps + 0.5);  // integer-stabilize across sensors
  if (sfps < 1.0) sfps = 1.0;
  if (update_rate >= sfps) return true;  // want >= server rate: every frame
  uint64_t b_now = static_cast<uint64_t>(
      std::floor(static_cast<double>(frame) * update_rate / sfps));
  uint64_t b_prev = static_cast<uint64_t>(
      std::floor(static_cast<double>(frame - 1) * update_rate / sfps));
  return b_now > b_prev;
}

}  // namespace carla_telemetry
