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
            name='arm_topic',
            default_value='arm_controller/command'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gripper_topic',
            default_value='gripper_controller/command'
        ),
        launch.actions.DeclareLaunchArgument(
            name='publishing_rate',
            default_value='50'
        ),
        launch_ros.actions.Node(
            package='morpheus_teleop',
            executable='trossen_joy.py',
            name='trossen_joy',
            output='screen',
            parameters=[
                {
                    'pose_topic': launch.substitutions.LaunchConfiguration('pose_topic')
                },
                {
                    'wrench_topic': launch.substitutions.LaunchConfiguration('wrench_topic')
                },
                {
                    'arm_topic': launch.substitutions.LaunchConfiguration('arm_topic')
                },
                {
                    'gripper_topic': launch.substitutions.LaunchConfiguration('gripper_topic')
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
