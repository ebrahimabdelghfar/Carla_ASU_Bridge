#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;

class SensorSyncTestNode : public rclcpp::Node {
public:
  SensorSyncTestNode() : Node("sensor_sync_test_node"), count_(0), sum_shift_(0.0), max_shift_(0.0), min_shift_(1e9) {
    this->declare_parameter<std::string>("config_path", "/home/ebrahim/asurt/config/carla_interface_config.yaml");
    std::string config_path = this->get_parameter("config_path").as_string();

    RCLCPP_INFO(this->get_logger(), "Reading config from %s", config_path.c_str());

    std::vector<std::string> camera_topics;
    std::vector<std::string> lidar_topics;
    std::string ns = "";

    try {
      YAML::Node config = YAML::LoadFile(config_path);
      if (config["ros2"] && config["ros2"]["namespace"]) {
        ns = "/" + config["ros2"]["namespace"].as<std::string>();
      }

      auto format_topic = [&](const std::string& t) {
        if (!t.empty() && t.front() == '/') return t;
        if (ns == "/") return ns + t;
        return ns + "/" + t;
      };

      if (config["cameras"]) {
        for (const auto& cam : config["cameras"]) {
          if (cam["enabled"] && cam["enabled"].as<bool>()) {
            camera_topics.push_back(format_topic(cam["topic_rgb"].as<std::string>()));
          }
        }
      }
      if (config["lidars"]) {
        for (const auto& lidar : config["lidars"]) {
          if (lidar["enabled"] && lidar["enabled"].as<bool>()) {
            lidar_topics.push_back(format_topic(lidar["topic_point_cloud"].as<std::string>()));
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read config: %s", e.what());
      return;
    }

    if (camera_topics.size() != 6 || lidar_topics.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Expected 6 cameras and at least 1 lidar! Found %zu cameras and %zu lidars", camera_topics.size(), lidar_topics.size());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Found 6 cameras and %zu lidars. Using first lidar.", lidar_topics.size());
    for (int i = 0; i < 6; ++i) {
      RCLCPP_INFO(this->get_logger(), "Camera %d: %s", i + 1, camera_topics[i].c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Lidar: %s", lidar_topics[0].c_str());

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

    sub_cam1_.subscribe(this, camera_topics[0], qos);
    sub_cam2_.subscribe(this, camera_topics[1], qos);
    sub_cam3_.subscribe(this, camera_topics[2], qos);
    sub_cam4_.subscribe(this, camera_topics[3], qos);
    sub_cam5_.subscribe(this, camera_topics[4], qos);
    sub_cam6_.subscribe(this, camera_topics[5], qos);
    sub_lidar_.subscribe(this, lidar_topics[0], qos);

    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(50), sub_cam1_, sub_cam2_, sub_cam3_, sub_cam4_, sub_cam5_, sub_cam6_, sub_lidar_));
    sync_->registerCallback(std::bind(&SensorSyncTestNode::syncCallback, this, _1, _2, _3, _4, _5, _6, _7));
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::PointCloud2>;

  void syncCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& msg1,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg2,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg3,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg4,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg5,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg6,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg7)
  {
    std::vector<rclcpp::Time> times = {
        msg1->header.stamp, msg2->header.stamp, msg3->header.stamp,
        msg4->header.stamp, msg5->header.stamp, msg6->header.stamp,
        msg7->header.stamp
    };

    double min_t = times[0].seconds();
    double max_t = times[0].seconds();
    for (const auto& t : times) {
      if (t.seconds() < min_t) min_t = t.seconds();
      if (t.seconds() > max_t) max_t = t.seconds();
    }

    double shift_ms = (max_t - min_t) * 1000.0;
    
    count_++;
    sum_shift_ += shift_ms;
    if (shift_ms > max_shift_) max_shift_ = shift_ms;
    if (shift_ms < min_shift_) min_shift_ = shift_ms;

    RCLCPP_INFO(this->get_logger(),
        "Synced [%zu] | Shift: %.3f ms | Metrics: Avg=%.3f ms, Max=%.3f ms, Min=%.3f ms",
        count_, shift_ms, sum_shift_ / count_, max_shift_, min_shift_);
  }

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam2_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam3_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam4_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam5_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam6_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;

  size_t count_;
  double sum_shift_;
  double max_shift_;
  double min_shift_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSyncTestNode>());
  rclcpp::shutdown();
  return 0;
}
