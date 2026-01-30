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
            default_value='fake'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_mode',
            default_value='interbotix'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='xbox360'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_keyboard',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/trossen_hera_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_model')
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/trossen.launch.py')
            ),
            launch_arguments={
                'robot_mode': launch.substitutions.LaunchConfiguration('robot_mode'),
                'control_mode': launch.substitutions.LaunchConfiguration('control_mode'),
                'controller': launch.substitutions.LaunchConfiguration('controller'),
                'use_keyboard': launch.substitutions.LaunchConfiguration('use_keyboard'),
                'xacro_path': launch.substitutions.LaunchConfiguration('xacro_path'),
                'srdf_path': launch.substitutions.LaunchConfiguration('srdf_path')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
