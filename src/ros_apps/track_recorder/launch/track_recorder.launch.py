import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('track_recorder'),
        'config',
        'track_recorder_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='track_recorder',
            executable='recorder_node',
            name='track_recorder',
            parameters=[config_file],
            output='screen'
        )
    ])
