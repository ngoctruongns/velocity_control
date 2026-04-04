import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('velocity_control'),
        'ros2_bridge',
        'ros2_bridge_params.yaml',
    )

    ros2_bridge_node = Node(
        package='velocity_control',
        executable='ros2_bridge',
        name='ros2_bridge_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        ros2_bridge_node,
    ])
