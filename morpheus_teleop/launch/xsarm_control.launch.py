import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_model',
            default_value='vx300s'
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
            name='show_ar_tag',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_gripper_bar',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_gripper_fingers',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_world_frame',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='external_urdf_loc',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='motor_configs',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_model')
        ),
        launch.actions.DeclareLaunchArgument(
            name='mode_configs',
            default_value=get_package_share_directory(
                'interbotix_xsarm_control') + '/config/modes.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='load_configs',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='load_description',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xs_sdk_type',
            default_value='xs_sdk'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xs_sdk_type',
            default_value='xs_sdk_sim'
        ),
        launch_ros.actions.Node(
            package='interbotix_xs_sdk',
            executable=launch.substitutions.LaunchConfiguration('xs_sdk_type'),
            name='xs_sdk',
            output='screen',
            parameters=[
                {
                    'motor_configs': launch.substitutions.LaunchConfiguration('motor_configs')
                },
                {
                    'mode_configs': launch.substitutions.LaunchConfiguration('mode_configs')
                },
                {
                    'load_configs': launch.substitutions.LaunchConfiguration('load_configs')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_description'), 'launch/xsarm_description.launch.py')
            ),
            launch_arguments={
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'base_link_frame': launch.substitutions.LaunchConfiguration('base_link_frame'),
                'show_ar_tag': launch.substitutions.LaunchConfiguration('show_ar_tag'),
                'show_gripper_bar': launch.substitutions.LaunchConfiguration('show_gripper_bar'),
                'show_gripper_fingers': launch.substitutions.LaunchConfiguration('show_gripper_fingers'),
                'use_world_frame': launch.substitutions.LaunchConfiguration('use_world_frame'),
                'external_urdf_loc': launch.substitutions.LaunchConfiguration('external_urdf_loc'),
                'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
