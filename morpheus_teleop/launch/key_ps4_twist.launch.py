import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('morpheus_teleop'), 'launch', 'ps4_twist.launch.py']
                )
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('morpheus_teleop'), 'launch', 'key_to_joy.launch.py']
                )
            )
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
