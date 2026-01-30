import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='morpheus_analysis',
            executable='morpheus_analysis',
            name='analysis_server',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
