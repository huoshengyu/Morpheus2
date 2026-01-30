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
            name='robot_mode',
            default_value='real'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_mode',
            default_value='none'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='none'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_keyboard',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='collision',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='collision_haptics',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawner',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory_name',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory_haptics',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/UCD_trossen_hera_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/trossen_hera_scene.srdf.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='data',
            default_value='true'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/trossen.launch.py')
            ),
            launch_arguments={
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'robot_mode': launch.substitutions.LaunchConfiguration('robot_mode'),
                'control_mode': launch.substitutions.LaunchConfiguration('control_mode'),
                'controller': launch.substitutions.LaunchConfiguration('controller'),
                'use_keyboard': launch.substitutions.LaunchConfiguration('use_keyboard'),
                'xacro_path': launch.substitutions.LaunchConfiguration('xacro_path'),
                'srdf_path': launch.substitutions.LaunchConfiguration('srdf_path')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_data'), 'launch/data.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
