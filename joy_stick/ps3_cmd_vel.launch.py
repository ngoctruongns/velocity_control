import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('velocity_control'),
        'joy_stick',
        'ps3_teleop.yaml',
    )

    utils_config_file = os.path.join(
        get_package_share_directory('velocity_control'),
        'joy_stick',
        'ps3_utils_control.yaml',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ],
    )

    utils_node = Node(
        package='velocity_control',
        executable='ps3_utils_control',
        name='ps3_utils_control_node',
        output='screen',
        parameters=[utils_config_file],
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        utils_node,
    ])
