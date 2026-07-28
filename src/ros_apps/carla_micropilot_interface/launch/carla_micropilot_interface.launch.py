from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments (overridable from CLI) ───────────────────────────
    max_rpm_arg = DeclareLaunchArgument(
        "max_rpm",
        default_value="200.0",
        description="Wheel RPM that maps to CARLA throttle 1.0"
    )

    max_steering_arg = DeclareLaunchArgument(
        "max_steering_angle_deg",
        default_value="16.0",
        description="Absolute steering limit in degrees (ICD: +left / -right)"
    )

    carla_role_arg = DeclareLaunchArgument(
        "carla_role_name",
        default_value="ego_vehicle",
        description="CARLA ego vehicle role name — must match carla_ros_bridge"
    )

    # ── Bridge node ───────────────────────────────────────────────────────
    bridge_node = Node(
        package="carla_micropilot_interface",
        executable="carla_micropilot_interface_node",
        name="carla_micropilot_interface_node",
        output="screen",
        parameters=[
            {
                # CLI arguments override config file values
                "max_rpm":                LaunchConfiguration("max_rpm"),
                "max_steering_angle_deg": LaunchConfiguration("max_steering_angle_deg"),
                "carla_role_name":        LaunchConfiguration("carla_role_name"),
            }
        ]
    )

    return LaunchDescription([
        max_rpm_arg,
        max_steering_arg,
        carla_role_arg,
        bridge_node,
    ])