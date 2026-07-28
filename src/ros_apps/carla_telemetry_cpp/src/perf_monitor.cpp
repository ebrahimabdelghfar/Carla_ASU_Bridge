#include "carla_telemetry/perf_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

namespace carla_telemetry {

namespace {

bool is_perf_enabled() {
  const char* val = std::getenv("CARLA_PERF");
  if (!val) return false;
  std::string s(val);
  return s != "0" && s != "" && s != "false";
}

double steady_now() {
  using clock = std::chrono::steady_clock;
  auto now = clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

// Nearest-rank percentile on an ascending-sorted, non-empty vector.
// rank = ceil(p/100 * n), clamped to [1, n]; return sorted[rank-1].
double percentile(const std::vector<double>& sorted, double p) {
  size_t n = sorted.size();
  size_t rank =
      static_cast<size_t>(std::ceil(p / 100.0 * static_cast<double>(n)));
  if (rank < 1) rank = 1;
  if (rank > n) rank = n;
  return sorted[rank - 1];
}

}  // namespace

PerfMonitor::PerfMonitor(bool enabled, double log_interval, size_t buffer_size)
    : enabled_(enabled),
      log_interval_(log_interval),
      buffer_size_(buffer_size),
      last_log_(steady_now()) {}

double PerfMonitor::tick() {
  // Must be monotonic: elapsed/interval deltas are meaningless if the clock can
  // jump backward. high_resolution_clock aliases system_clock on libstdc++
  // (non-steady) → NTP/clock adjustments produce negative or huge samples.
  using clock = std::chrono::steady_clock;
  auto now = clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

void PerfMonitor::record(const std::string& name, double start) {
  if (!enabled_) return;
  double elapsed_ms = (tick() - start) * 1000.0;
  {
    std::lock_guard<std::mutex> lk(lock_);
    auto& buf = metrics_[name];
    buf.push_back(elapsed_ms);
    if (buf.size() > buffer_size_) buf.pop_front();
  }
  maybe_log();
}

void PerfMonitor::record_value(const std::string& name, double value) {
  if (!enabled_) return;
  std::lock_guard<std::mutex> lk(lock_);
  auto& buf = metrics_[name];
  buf.push_back(value);
  if (buf.size() > buffer_size_) buf.pop_front();
}

void PerfMonitor::record_interval(const std::string& name) {
  if (!enabled_) return;
  double now = tick();
  bool have_prev = false;
  {
    std::lock_guard<std::mutex> lk(lock_);
    auto it = last_ts_.find(name);
    if (it != last_ts_.end()) {
      double dt_ms = (now - it->second) * 1000.0;
      auto& buf = metrics_[name];
      buf.push_back(dt_ms);
      if (buf.size() > buffer_size_) buf.pop_front();
      have_prev = true;
    }
    last_ts_[name] = now;
  }
  if (have_prev) maybe_log();
}

void PerfMonitor::maybe_log() {
  double now = steady_now();
  if ((now - last_log_) < log_interval_) return;

  std::unordered_map<std::string, std::vector<double>> snapshot;
  {
    std::lock_guard<std::mutex> lk(lock_);
    if ((now - last_log_) < log_interval_) return;
    last_log_ = now;
    for (auto& [k, v] : metrics_) {
      snapshot[k] = std::vector<double>(v.begin(), v.end());
    }
    // Reset the window: each report covers ONLY samples since the last one.
    // Without this the deque retained up to buffer_size_ (2000) samples, so
    // avg/percentiles were a long cumulative mean that barely moved report
    // to report ("constant average"). last_ts_ is untouched → record_interval
    // spacing stays continuous across reports.
    metrics_.clear();
  }

  if (snapshot.empty()) return;

  std::ostringstream ss;
  ss << "\n╔══ PERF REPORT (last " << static_cast<int>(log_interval_)
     << "s) ═══════════════════════════════════╗\n";
  ss << "║  cols = ms | p50 = median | p95 = 95% of samples ≤ value "
        "(tail/jitter)\n";
  ss << "║  *.server_dt = server → client interval (server Hz ≈ 1000/avg)\n";

  std::vector<std::string> keys;
  keys.reserve(snapshot.size());
  for (auto& [k, _] : snapshot) keys.push_back(k);
  std::sort(keys.begin(), keys.end());

  for (auto& name : keys) {
    auto vals = snapshot[name];
    if (vals.empty()) continue;
    std::sort(vals.begin(), vals.end());
    size_t n = vals.size();
    double p50 = percentile(vals, 50.0);
    double p95 = percentile(vals, 95.0);
    double mx = vals.back();
    double avg =
        std::accumulate(vals.begin(), vals.end(), 0.0) / static_cast<double>(n);
    ss << "║  " << std::left << std::setw(40) << name << "  n=" << std::setw(5)
       << n << "  avg=" << std::fixed << std::setprecision(2) << std::setw(7)
       << avg << "  p50=" << std::setw(7) << p50 << "  p95=" << std::setw(7)
       << p95 << "  max=" << std::setw(7) << mx << " ms\n";
  }
  ss << "╚════════════════════════════════════════════════════════╝";
  std::cerr << ss.str() << std::endl;
}

// Module-level singleton
PerfMonitor perf(is_perf_enabled());

}  // namespace carla_telemetry
