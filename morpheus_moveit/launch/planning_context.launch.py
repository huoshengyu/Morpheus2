import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='load_robot_description',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/test_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/test_scene.srdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='joint_limits_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/joint_limits.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cartesian_limits_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/cartesian_limits.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='kinematics_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/ur5e_kinematics.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_description',
            default_value='robot_description'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
