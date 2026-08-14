#pragma once

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace carla_telemetry {

// Lightweight pipeline performance instrumentation.
// Enable via the CARLA_PERF=1 env var; all calls become no-ops when disabled.
class PerfMonitor {
 public:
  explicit PerfMonitor(bool enabled = true, double log_interval = 10.0,
                       size_t buffer_size = 2000);

  // Capture a start timestamp (always cheap).
  static double tick();

  // Record elapsed time since `start` under metric `name`.
  void record(const std::string& name, double start);

  // Record an arbitrary value (e.g. queue depth).
  void record_value(const std::string& name, double value);

  // Record the interval (ms) since the previous call under `name`.
  // Measures the rate the CARLA server delivers data to the client
  // (inter-arrival time; server Hz = 1000 / avg). The first call only primes
  // the timer.
  void record_interval(const std::string& name);

 private:
  void maybe_log();

  bool enabled_;
  double log_interval_;
  size_t buffer_size_;
  std::mutex lock_;
  std::unordered_map<std::string, std::deque<double>>
      metrics_;                                      // Metrics to be logged
  std::unordered_map<std::string, double> last_ts_;  // for record_interval
  double last_log_;
};

// Module-level singleton — enabled via CARLA_PERF env var.
extern PerfMonitor perf;

}  // namespace carla_telemetry
