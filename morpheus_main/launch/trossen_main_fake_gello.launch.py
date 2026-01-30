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
            default_value='gello'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='gello'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_keyboard',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='collision',
            default_value='true'
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
                'morpheus_description') + '/urdf/scenes/trossen_hera_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/trossen_hera_scene.srdf.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='data',
            default_value='false'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_main'), 'launch/trossen_main.launch.py')
            ),
            launch_arguments={
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'robot_mode': launch.substitutions.LaunchConfiguration('robot_mode'),
                'control_mode': launch.substitutions.LaunchConfiguration('control_mode'),
                'controller': launch.substitutions.LaunchConfiguration('controller'),
                'use_keyboard': 'false',
                'xacro_path': launch.substitutions.LaunchConfiguration('xacro_path'),
                'srdf_path': launch.substitutions.LaunchConfiguration('srdf_path'),
                'data': launch.substitutions.LaunchConfiguration('data'),
                'collision': launch.substitutions.LaunchConfiguration('collision'),
                'collision_haptics': launch.substitutions.LaunchConfiguration('collision_haptics'),
                'spawner': launch.substitutions.LaunchConfiguration('spawner'),
                'trajectory': launch.substitutions.LaunchConfiguration('trajectory'),
                'trajectory_name': launch.substitutions.LaunchConfiguration('trajectory_name'),
                'trajectory_haptics': launch.substitutions.LaunchConfiguration('trajectory_haptics')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'gello'), 'launch/gello.launch.py')
            ),
            launch_arguments={
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'robot': 'trossen',
                'agent': 'gello',
                'gello_port': 'fake_trossen',
                'use_keyboard': launch.substitutions.LaunchConfiguration('use_keyboard')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
