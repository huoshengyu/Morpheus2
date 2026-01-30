import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='twist_topic',
            default_value='twist_controller/command'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pose_topic',
            default_value='target_frame'
        ),
        launch.actions.DeclareLaunchArgument(
            name='wrench_topic',
            default_value='target_wrench'
        ),
        launch.actions.DeclareLaunchArgument(
            name='frame_id',
            default_value='vx300s/base_link'
        ),
        launch.actions.DeclareLaunchArgument(
            name='end_effector',
            default_value='vx300s/ee_gripper_link'
        ),
        launch.actions.DeclareLaunchArgument(
            name='publishing_rate',
            default_value='50'
        ),
        launch_ros.actions.Node(
            package='morpheus_teleop',
            executable='twist_converter.py',
            name='twist_converter',
            output='screen',
            parameters=[
                {
                    'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic')
                },
                {
                    'pose_topic': launch.substitutions.LaunchConfiguration('pose_topic')
                },
                {
                    'wrench_topic': launch.substitutions.LaunchConfiguration('wrench_topic')
                },
                {
                    'frame_id': launch.substitutions.LaunchConfiguration('frame_id')
                },
                {
                    'end_effector': launch.substitutions.LaunchConfiguration('end_effector')
                },
                {
                    'publishing_rate': launch.substitutions.LaunchConfiguration('publishing_rate')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
