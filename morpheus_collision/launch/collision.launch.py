import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gripper_group',
            default_value='gripper'
        ),
        launch.actions.DeclareLaunchArgument(
            name='collision_haptics',
            default_value=launch.substitutions.LaunchConfiguration(
                'collision_haptics')
        ),
        launch_ros.actions.Node(
            package='morpheus_collision',
            executable='morpheus_collision',
            name='collision',
            output='screen',
            parameters=[
                {
                    'arm_group': launch.substitutions.LaunchConfiguration('arm_group')
                },
                {
                    'gripper_group': launch.substitutions.LaunchConfiguration('gripper_group')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_collision'), 'launch/robot_link_vector.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_collision'), 'launch/allowed_collision_vector.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_haptics'), 'launch/collision_send.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
