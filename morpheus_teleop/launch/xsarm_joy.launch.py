import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_model',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_name',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_model')
        ),
        launch.actions.DeclareLaunchArgument(
            name='base_link_frame',
            default_value='base_link'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mode_configs',
            default_value=get_package_share_directory(
                'interbotix_xsarm_joy') + '/config/modes.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='threshold',
            default_value='0.75'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='ps4'
        ),
        launch.actions.DeclareLaunchArgument(
            name='launch_driver',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_keyboard',
            default_value='false'
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
            executable='xsarm_joy',
            name='xsarm_joy',
            output='screen',
            parameters=[
                {
                    'threshold': launch.substitutions.LaunchConfiguration('threshold')
                },
                {
                    'controller': launch.substitutions.LaunchConfiguration('controller')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='morpheus_teleop',
            executable='xsarm_robot.py',
            name='xsarm_robot',
            output='screen',
            parameters=[
                {
                    'robot_model': launch.substitutions.LaunchConfiguration('robot_model')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/xsarm_control.launch.py')
            ),
            launch_arguments={
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'base_link_frame': launch.substitutions.LaunchConfiguration('base_link_frame'),
                'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz'),
                'mode_configs': launch.substitutions.LaunchConfiguration('mode_configs'),
                'use_sim': launch.substitutions.LaunchConfiguration('use_sim')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
