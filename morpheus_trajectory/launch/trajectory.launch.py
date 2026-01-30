import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='trajectory_name',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='goal_name_vector',
            default_value='trossen_home hera_ceiling hera_floor hera_crouch_front hera_crouch_right hera_arch_over hera_arch_pick'
        ),
        launch.actions.DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gripper_group',
            default_value='gripper'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mode',
            default_value='preset'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='ps4'
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory_haptics',
            default_value='false'
        ),
        launch_ros.actions.Node(
            package='morpheus_trajectory',
            executable='serial_trajec_sender',
            name='serial_trajec_sender',
            output='screen',
            parameters=[
                {
                    'goal/name_vector': launch.substitutions.LaunchConfiguration('goal_name_vector')
                },
                {
                    'goal/trajectory_name': launch.substitutions.LaunchConfiguration('trajectory_name')
                }
            ],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('trajectory_haptics'))
        ),
        launch_ros.actions.Node(
            package='morpheus_trajectory',
            executable='morpheus_trajectory',
            name='trajectory',
            output='screen',
            parameters=[
                {
                    'goal/name_vector': launch.substitutions.LaunchConfiguration('goal_name_vector')
                },
                {
                    'goal/trajectory_name': launch.substitutions.LaunchConfiguration('trajectory_name')
                },
                {
                    'arm_group': launch.substitutions.LaunchConfiguration('arm_group')
                },
                {
                    'gripper_group': launch.substitutions.LaunchConfiguration('gripper_group')
                },
                {
                    'mode': launch.substitutions.LaunchConfiguration('mode')
                },
                {
                    'controller': launch.substitutions.LaunchConfiguration('controller')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
