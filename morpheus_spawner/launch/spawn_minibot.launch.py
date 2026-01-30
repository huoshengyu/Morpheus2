import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_name'
        ),
        launch.actions.DeclareLaunchArgument(
            name='init_pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_description'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_minibot_model',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
