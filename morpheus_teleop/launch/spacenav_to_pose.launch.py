import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='twist_topic',
            default_value='/twist_controller/command'
        ),
        launch_ros.actions.Node(
            package='cartesian_controller_utilities',
            executable='pose.py',
            name='spacenav_to_pose',
            output='screen',
            parameters=[
                {
                    'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic')
                },
                {
                    'pose_topic': '/target_frame'
                },
                {
                    'frame_id': 'base_link'
                },
                {
                    'end_effector': 'tcp_link'
                },
                {
                    'publishing_rate': '50'
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
