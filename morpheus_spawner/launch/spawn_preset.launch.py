import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='preset_name',
            default_value='dragon'
        ),
        launch.actions.DeclareLaunchArgument(
            name='namespace',
            default_value='/'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
