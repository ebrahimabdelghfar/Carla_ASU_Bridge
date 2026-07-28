"""
Launch file for the CARLA telemetry interface.

Usage:
    ros2 launch carla_telemetry_cpp carla_telemetry.launch.py
    ros2 launch carla_telemetry_cpp carla_telemetry.launch.py \\
        config_file:=/path/to/carla_interface_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, EmitEvent, RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import launch_ros.events.lifecycle
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    # Installed at: install/ros_apps/carla_telemetry_cpp/share/carla_telemetry_cpp/launch/
    # Need 6 levels up to reach workspace root
    project_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "..")
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

    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Automatically configure and activate the lifecycle node",
    )

    # ── Main telemetry node ───────────────────────────────────────────
    telemetry_node = LifecycleNode(
        package="carla_telemetry_cpp",
        executable="carla_telemetry_node",
        name="micropilot_carla_bridge_node",
        namespace="",
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

    emit_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('micropilot_carla_bridge_node'),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration("auto_start"))
    )

    emit_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=telemetry_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('micropilot_carla_bridge_node'),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start"))
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=["0","0","0","0","0","0","map","odom"]
    )

    # Enable shared-memory transport so the large camera/LiDAR payloads move
    # over /dev/shm instead of loopback sockets. Set BEFORE the node process
    # starts and independently of the launching shell (make recipes and some
    # service contexts do not source ~/.bashrc). Works with either RMW:
    #   rmw_zenoh_cpp     → ZENOH_CONFIG_OVERRIDE turns on Zenoh SHM.
    #   rmw_cyclonedds_cpp → CYCLONEDDS_URI points at a config with Iceoryx SHM
    #                        (requires an iox-roudi daemon running on the host).
    # An explicit external value always wins (skipped if already set).
    rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    pre_actions = []
    if "zenoh" in rmw:
        if "ZENOH_CONFIG_OVERRIDE" not in os.environ:
            pre_actions.append(SetEnvironmentVariable(
                "ZENOH_CONFIG_OVERRIDE",
                "transport/shared_memory/enabled=true;"
                "transport/unicast/lowlatency=false"))
    else:
        if "CYCLONEDDS_URI" not in os.environ:
            pre_actions.append(SetEnvironmentVariable(
                "CYCLONEDDS_URI",
                "file://" + os.path.expanduser("~/.config/cyclonedds/cyclonedds.xml")))

    return LaunchDescription(pre_actions + [
        config_file_arg,
        open_manual_control_arg,
        log_level_arg,
        auto_start_arg,
        LogInfo(msg=["Launching CARLA Telemetry Node with config: ",
                     LaunchConfiguration("config_file")]),
        telemetry_node,
        map_to_odom,
        emit_configure,
        emit_activate
    ])
