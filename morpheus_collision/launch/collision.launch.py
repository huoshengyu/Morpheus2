import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        DeclareLaunchArgument(
            name='gripper_group',
            default_value='gripper'
        ),
        Node(
            package='morpheus_collision',
            executable='collision',
            name='collision',
            output='screen',
            parameters=[
                {
                    'arm_group': LaunchConfiguration('arm_group')
                },
                {
                    'gripper_group': LaunchConfiguration('gripper_group')
                }
            ]
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
