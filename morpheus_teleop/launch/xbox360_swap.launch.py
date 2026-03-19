import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='twist_topic',
            default_value='twist_controller/command'
        ),
        launch.actions.DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        launch.actions.DeclareLaunchArgument(
            name='frame_id',
            default_value='base_link'
        ),
        launch.actions.DeclareLaunchArgument(
            name='end_effector',
            default_value='tcp_link'
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {
                    'dev': '/dev/input/js0'
                },
                {
                    'dev_ff': '/dev/input/event7'
                },
                {
                    'default_trig_val': 'true'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='morpheus_teleop',
            executable='xbox360_swap.py',
            name='xbox360_twist',
            output='screen',
            parameters=[
                {
                    'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic')
                },
                {
                    'arm_group': launch.substitutions.LaunchConfiguration('arm_group')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/twist_to_pose.launch.py')
            ),
            launch_arguments={
                'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic'),
                'frame_id': launch.substitutions.LaunchConfiguration('frame_id'),
                'end_effector': launch.substitutions.LaunchConfiguration('end_effector')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
