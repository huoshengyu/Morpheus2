import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_file',
            default_value='bar_12in'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_name',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_file')
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_pose',
            default_value='-x -0.1 -y 0.8528 -z 0.91915 -R 1.57079633 -P 0.0 -Y 0',
            description='Pose to spawn the robot at'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
