"""
Launch file for the CARLA telemetry interface.

Usage:
    ros2 launch carla_telemetry carla_telemetry.launch.py
    ros2 launch carla_telemetry carla_telemetry.launch.py \\
        config_file:=/path/to/carla_interface_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    project_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
    )
    default_config = os.path.join(
        project_root, "config", "carla_interface_config.yaml"
    )

    # ── Launch arguments ──────────────────────────────────────────────
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Absolute path to carla_interface_config.yaml",
    )

    open_manual_control_arg = DeclareLaunchArgument(
        "open_manual_control",
        default_value="",
        description="Override whether to open manual control ('true' or 'false', empty defaults to config file value)",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 log level: debug | info | warn | error | fatal",
    )

    # ── Main telemetry node ───────────────────────────────────────────
    telemetry_node = Node(
        package="carla_telemetry",
        executable="carla_telemetry_node",
        name="carla_telemetry_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "open_manual_control": LaunchConfiguration("open_manual_control"),
            }
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        open_manual_control_arg,
        log_level_arg,
        LogInfo(msg=["Launching CARLA Telemetry Node with config: ",
                     LaunchConfiguration("config_file")]),
        telemetry_node,
    ])
