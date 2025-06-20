from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robocon_pkg_share = get_package_share_directory('robocon_2025')

    mcu_control_node = Node(
        package='Locomotion_ROS',
        executable='mcu_control',
        output='screen'
    )

    joystick_control_node = Node(
        package='Locomotion_ROS',
        executable='joystick_control',
        output='screen'
    )

    return LaunchDescription([
        mcu_control_node,
        joystick_control_node
    ])