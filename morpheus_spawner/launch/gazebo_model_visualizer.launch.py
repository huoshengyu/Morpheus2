import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='morpheus_spawner',
            executable='morpheus_spawner',
            name='gazebo_model_visualizer',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
