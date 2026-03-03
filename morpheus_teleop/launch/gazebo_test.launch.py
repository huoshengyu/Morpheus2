import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/a_bot_double_sided_shelf_scene.xacro'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_moveit'), 'launch/a_bot_gazebo.launch.py')
            ),
            launch_arguments={
                'urdf_path': launch.substitutions.LaunchConfiguration('urdf_path'),
                'use_rviz': 'true'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
