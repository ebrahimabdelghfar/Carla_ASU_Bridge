#include "carla_micropilot_interface/carla_micropilot_interface_node.hpp"
#include <algorithm>
#include <cmath>
#include <string>

// Macros for cleaner topic creation
#define SUB(topic, msg, qos, callback) this->create_subscription<msg>(topic, qos, callback);
#define PUB(topic, msg, qos) this->create_publisher<msg>(topic, qos);

CarlaMicropilotInterfaceNode::CarlaMicropilotInterfaceNode(
    const rclcpp::NodeOptions &options)
    : Node("carla_micropilot_interface_node", options)
{
  //! ── Declare/Read parameters
  //! ──────────────────────────────────────────────────
  this->declare_parameter<double>("max_rpm", 200.0);
  this->declare_parameter<double>("max_steering_angle_deg", 16.0);
  this->declare_parameter<std::string>("carla_role_name", "hero");

  _max_rpm = this->get_parameter("max_rpm").as_double();
  _max_steering_angle_deg =
      this->get_parameter("max_steering_angle_deg").as_double();
  _carla_role_name = this->get_parameter("carla_role_name").as_string();

  RCLCPP_INFO(this->get_logger(),
              "carla_micropilot_interface started | max_rpm=%.1f | "
              "max_steer=±%.1f° | role='%s'",
              _max_rpm, _max_steering_angle_deg, _carla_role_name.c_str());

  const std::string carla_ns = "/carla/" + _carla_role_name;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  //! ── Publishers ───────────────────────────────────────────────────────────

  // micropilot → CARLA: vehicle control command
  std::string topic = carla_ns + "/vehicle_control_cmd";
  _carla_control_pub = PUB(topic, carla_msgs::msg::CarlaEgoVehicleControl, qos);

  // carla → micropilot: speed feedback (m/s)
  topic = "/sim/feedback/speed";
  _sim_speed_pub = PUB(topic, std_msgs::msg::Float32, qos);

  // carla → micropilot: steering angle feedback (degrees)
  topic = "/sim/feedback/steering_angle";
  _sim_steering_pub = PUB(topic, std_msgs::msg::Float32, qos);

  //! ── Subscribers ──────────────────────────────────────────────────────────

  // ← micropilot: velocity command in RPM
  topic = "/sim/control/velocity_rpm";
  _velocity_rpm_sub = SUB(topic, std_msgs::msg::Float32, qos, [&](const std_msgs::msg::Float32::SharedPtr msg)
                          {
                            _current_velocity_rpm = msg->data;
                            _sim_speed_pub->publish(*msg);
                            publishCarlaControl();
                          });

  // ← SimulationTransport: steering angle command in degrees
  topic = "/sim/control/steering_angle_deg";
  _steering_deg_sub = SUB(topic, std_msgs::msg::Float32, qos,
                          [&](const std_msgs::msg::Float32::SharedPtr msg)
                          {
                            _current_steering_deg = msg->data;
                            _sim_steering_pub->publish(*msg);
                            publishCarlaControl();
                          });

  // ← SimulationTransport: brake command (bool)
  topic = "/sim/control/brake";
  _brake_sub = SUB(topic, std_msgs::msg::Bool, qos,
                   [&](const std_msgs::msg::Bool::SharedPtr msg)
                   {
                     _brake_engaged = msg->data;
                     publishCarlaControl();
                   });

  // ← CARLA: speedometer (m/s) — forward to /sim/feedback/speed
  topic = carla_ns + "/speedometer";
  _speedometer_sub = SUB(topic, std_msgs::msg::Float32, qos,
                         [&](const std_msgs::msg::Float32::SharedPtr msg)
                         {
                           _sim_speed_pub->publish(*msg);
                         });
}

void CarlaMicropilotInterfaceNode::publishCarlaControl()
{
  auto cmd = carla_msgs::msg::CarlaEgoVehicleControl();
  cmd.header.stamp = this->now();

  if (_brake_engaged)
  {
    cmd.throttle = 0.0f;
    cmd.steer = 0.0f;
    cmd.brake = 1.0f;
    cmd.hand_brake = true;
    cmd.reverse = false;
    cmd.manual_gear_shift = true;
    cmd.gear = 1;
  }
  else
  {
    cmd.throttle = rpmToThrottle(_current_velocity_rpm);
    cmd.steer = steeringDegToCarla(_current_steering_deg);
    cmd.brake = 0.0f;
    cmd.hand_brake = false;
    cmd.reverse = (_current_velocity_rpm < 0.0f);
    cmd.manual_gear_shift = true;
    cmd.gear = (_current_velocity_rpm < 0.0f) ? -1 : 1;
  }
  RCLCPP_INFO(this->get_logger(), "Publishing carla control command: throttle=%.2f, steer=%.2f, brake=%.2f, hand_brake=%s, reverse=%s", cmd.throttle, cmd.steer, cmd.brake, cmd.hand_brake ? "true" : "false", cmd.reverse ? "true" : "false");
  _carla_control_pub->publish(cmd);
}

float CarlaMicropilotInterfaceNode::rpmToThrottle(float rpm) const
{
  if (_max_rpm <= 0.0)
  {
    return 0.0f;
  }

  // Support reverse: map negative RPM to reverse throttle
  const float abs_rpm = std::abs(rpm);
  const float throttle = static_cast<float>(abs_rpm / _max_rpm);
  return std::clamp(throttle, 0.0f, 1.0f);
}

float CarlaMicropilotInterfaceNode::steeringDegToCarla(float deg) const
{
  if (_max_steering_angle_deg <= 0.0)
  {
    return 0.0f;
  }

  // ICD: +deg = left, -deg = right
  // CARLA: +steer = right, -steer = left  → invert sign
  const float steer = static_cast<float>(-deg / _max_steering_angle_deg);
  return std::clamp(steer, -1.0f, 1.0f);
}
