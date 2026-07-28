// DDS profiler node — consumer-side view of the bridge's ROS 2 / CycloneDDS
// output. Subscribes to the sensor + feedback topics defined in the SAME config
// the bridge uses, and reports per-topic received rate and end-to-end latency
// through the shared PerfMonitor boxed report (stderr, every 10s).
//
//   dds.<topic>.recv_dt   = inter-arrival interval (ms); received Hz = 1000/avg
//   dds.<topic>.latency_ms = header.stamp → arrival age (ms)
//
// Subscriptions use BEST_EFFORT reliability: a BEST_EFFORT reader is QoS-
// compatible with BOTH reliable and best-effort writers, so the profiler
// receives every topic regardless of the publisher's setting. Drop estimate =
// compare received Hz (recv_dt) against the expected update_rate logged at
// startup; BEST_EFFORT depth-limited drops are silent and carry no sequence
// number, so true per-sample gap detection is out of scope.

#include <yaml-cpp/yaml.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "carla_telemetry/perf_monitor.hpp"

namespace carla_telemetry {

class DdsProfiler : public rclcpp::Node {
 public:
  DdsProfiler() : rclcpp::Node("dds_profiler"), profiler_(/*enabled=*/true) {
    std::string cfg_path = declare_parameter<std::string>("config_file", "");
    if (cfg_path.empty())
      cfg_path = "config/carla_interface_config.yaml";  // repo-root fallback

    YAML::Node cfg;
    try {
      cfg = YAML::LoadFile(cfg_path);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(),
                   "cannot load config '%s': %s "
                   "(pass -p config_file:=<abs path>)",
                   cfg_path.c_str(), e.what());
      throw;
    }

    ns_ = cfg["ros2"] ? cfg["ros2"]["namespace"].as<std::string>("sim") : "sim";

    // Cameras (sensor_msgs/Image) — topics are absolute in config.
    if (cfg["cameras"]) {
      for (const auto& c : cfg["cameras"]) {
        if (!c["enabled"].as<bool>(false)) continue;
        subscribe<sensor_msgs::msg::Image>(
            resolve(c["topic_rgb"].as<std::string>("")),
            c["update_rate"].as<double>(10.0));
      }
    }
    if (cfg["third_person_view"] &&
        cfg["third_person_view"]["enabled"].as<bool>(false)) {
      const auto& t = cfg["third_person_view"];
      subscribe<sensor_msgs::msg::Image>(
          resolve(t["topic_rgb"].as<std::string>("")),
          t["update_rate"].as<double>(10.0));
    }

    // LiDARs (sensor_msgs/PointCloud2).
    if (cfg["lidars"]) {
      for (const auto& l : cfg["lidars"]) {
        if (!l["enabled"].as<bool>(false)) continue;
        subscribe<sensor_msgs::msg::PointCloud2>(
            resolve(l["topic_point_cloud"].as<std::string>("")),
            l["update_rate"].as<double>(10.0));
      }
    }

    // Feedback (header-bearing) — topics relative, resolved under namespace.
    const auto& topics = cfg["ros2"] ? cfg["ros2"]["topics"] : YAML::Node();
    auto tp = [&](const char* key, const char* def) {
      return resolve(topics && topics[key] ? topics[key].as<std::string>(def)
                                           : std::string(def));
    };
    if (cfg["imu"])
      subscribe<sensor_msgs::msg::Imu>(
          tp("feedback_imu", "feedback/imu"),
          cfg["imu"]["update_rate"].as<double>(50.0));
    if (cfg["gps"])
      subscribe<sensor_msgs::msg::NavSatFix>(
          tp("feedback_gps", "feedback/gps"),
          cfg["gps"]["update_rate"].as<double>(10.0));
    if (cfg["odometry"])
      subscribe<nav_msgs::msg::Odometry>(
          tp("odom", "odom"), cfg["odometry"]["update_rate"].as<double>(50.0));

    if (subs_.empty())
      RCLCPP_WARN(get_logger(), "no enabled topics found in %s",
                  cfg_path.c_str());
    else
      RCLCPP_INFO(get_logger(),
                  "profiling %zu topics — PERF REPORT to stderr every 10s",
                  subs_.size());
  }

 private:
  // Same resolution rule as CarlaROS2Backend::topic().
  std::string resolve(const std::string& suffix) const {
    if (suffix.empty()) return "/";
    if (suffix[0] == '/') return suffix;
    if (ns_.empty()) return "/" + suffix;
    return "/" + ns_ + "/" + suffix;
  }

  template <typename MsgT>
  void subscribe(const std::string& topic, double expected_hz) {
    if (topic.empty() || topic == "/") return;
    const std::string recv = "dds." + topic + ".recv_dt";
    const std::string lat = "dds." + topic + ".latency_ms";
    auto sub = create_subscription<MsgT>(
        topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this, recv, lat](const typename MsgT::SharedPtr msg) {
          profiler_.record_interval(recv);
          rclcpp::Time stamp(msg->header.stamp);
          if (stamp.nanoseconds() > 0) {
            double age_ms = (this->now() - stamp).seconds() * 1000.0;
            profiler_.record_value(lat, age_ms);
          }
        });
    subs_.push_back(sub);
    RCLCPP_INFO(get_logger(), "  %-32s expected %.1f Hz", topic.c_str(),
                expected_hz);
  }

  std::string ns_;
  PerfMonitor profiler_;  // always-on, own instance (own report window)
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

}  // namespace carla_telemetry

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carla_telemetry::DdsProfiler>());
  rclcpp::shutdown();
  return 0;
}
