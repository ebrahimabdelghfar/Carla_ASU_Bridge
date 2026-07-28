#ifndef __CARLA_MICROPILOT_INTERFACE_HPP__
#define __CARLA_MICROPILOT_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

/**
 * @brief Node for interfacing between micropilot and CARLA
 *
 *  Bridge node between micropilot_vehicle_interface SimulationTransport
 * and the CARLA ROS 2 bridge.
 */
class CarlaMicropilotInterfaceNode : public rclcpp::Node
{
public:
    explicit CarlaMicropilotInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // ── Parameter cache ──────────────────────────────────────────────────────
    double _max_rpm;                ///< RPM that maps to throttle 1.0
    double _max_steering_angle_deg; ///< Absolute steering limit in degrees (positive value)
    std::string _carla_role_name;   ///< CARLA ego vehicle role name (default: "hero")

    // ── Internal state (last received values) ────────────────────────────────
    float _current_velocity_rpm{0.0f};
    float _current_steering_deg{0.0f};
    bool _brake_engaged{false};

    // ── Publishers ───────────────────────────────────────────────────────────
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _carla_control_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _sim_speed_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _sim_steering_pub;

    // ── Subscribers ──────────────────────────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _velocity_rpm_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _steering_deg_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _brake_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _speedometer_sub;

    // ── Callbacks ────────────────────────────────────────────────────────────
    void onVelocityRpm(const std_msgs::msg::Float32::SharedPtr msg);
    void onSteeringDeg(const std_msgs::msg::Float32::SharedPtr msg);
    void onBrake(const std_msgs::msg::Bool::SharedPtr msg);
    void onSpeedometer(const std_msgs::msg::Float32::SharedPtr msg);

    // ── Helpers ───────────────────────────────────────────────────────────────
    void publishCarlaControl();

    /// Linear map: rpm in [0, max_rpm_] → throttle in [0.0, 1.0]
    float rpmToThrottle(float rpm) const;

    /// Linear map: deg in [-max_steering_angle_deg_, +max_steering_angle_deg_]
    ///             → steer in [-1.0, +1.0]
    /// NOTE: CARLA convention — positive steer = right, negative = left.
    ///       Your ICD: +deg = left, -deg = right → we invert the sign.
    float steeringDegToCarla(float deg) const;
};

#endif //__CARLA_MICROPILOT_INTERFACE_HPP__
