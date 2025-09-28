from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Các tham số robot
    wheel_radius = DeclareLaunchArgument("wheel_radius", default_value="0.05")
    wheel_base   = DeclareLaunchArgument("wheel_base", default_value="0.30")
    ticks_per_rev= DeclareLaunchArgument("ticks_per_rev", default_value="1024")
    port         = DeclareLaunchArgument("port", default_value="/dev/ttyUSB0")
    baudrate     = DeclareLaunchArgument("baudrate", default_value="115200")

    # Node encoder odometry
    encoder_node = Node(
        package="my_robot_pkg",
        executable="encoder_odom_node",
        name="encoder_odom_node",
        parameters=[{
            "wheel_radius": LaunchConfiguration("wheel_radius"),
            "wheel_base":   LaunchConfiguration("wheel_base"),
            "ticks_per_rev":LaunchConfiguration("ticks_per_rev"),
            "port":         LaunchConfiguration("port"),
            "baudrate":     LaunchConfiguration("baudrate")
        }],
        output="screen"
    )

    # # Robot state publisher (đọc URDF và publish tf tĩnh)
    # urdf_file = os.path.join(
    #     os.getenv("AMENT_PREFIX_PATH").split(":")[0],
    #     "share", "my_robot_pkg", "urdf", "my_robot.urdf"
    # )
    # with open(urdf_file, "r") as infp:
    #     robot_desc = infp.read()

    # rsp_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_desc}],
    #     output="screen"
    # )

    ld.add_action(wheel_radius)
    ld.add_action(wheel_base)
    ld.add_action(ticks_per_rev)
    ld.add_action(port)
    ld.add_action(baudrate)
    ld.add_action(encoder_node)
    # ld.add_action(rsp_node)

    return ld
