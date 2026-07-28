#pragma once

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace carla_telemetry {

/**
 * @brief Lightweight pipeline performance instrumentation.
 * @details Enable via CARLA_PERF=1 env var. All calls become no-ops when
 * disabled.
 */
class PerfMonitor {
 public:
  /**
   * @brief Construct a new PerfMonitor object.
   * @param enabled
   * @param log_interval
   * @param buffer_size
   */
  explicit PerfMonitor(bool enabled = true, double log_interval = 10.0,
                       size_t buffer_size = 2000);

  /**
   * @brief Capture a start timestamp (always cheap).
   *
   * @return double The start timestamp
   */
  static double tick();

  /**
   * @brief Record elapsed time since `start` under metric `name`.
   *
   * @param name
   * @param start
   */
  void record(const std::string& name, double start);

  /**
   * @brief Record an arbitrary value (e.g. queue depth).
   *
   * @param name
   * @param value
   */
  void record_value(const std::string& name, double value);

  /**
   * @brief Record the interval (ms) since the previous call under `name`.
   * @details Measures the rate the CARLA server delivers data to the client
   * (inter-arrival time; server Hz = 1000 / avg). First call primes only.
   * @param name
   */
  void record_interval(const std::string& name);

 private:
  /**
   * @brief Log the current metrics.
   */
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

/// Module-level singleton — enabled via CARLA_PERF env var.
extern PerfMonitor perf;

}  // namespace carla_telemetry
