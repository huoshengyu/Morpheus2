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
            name='load_gazebo_configs',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_joint_pub',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_joint_pub_gui',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rate',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='source_list',
            default_value='[]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory(
                'interbotix_xsarm_descriptions') + '/rviz/xsarm_description.rviz'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_model')
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {
                    '$(arg robot_name)/robot_description': None
                },
                {
                    'rate': launch.substitutions.LaunchConfiguration('rate')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[
                {
                    '$(arg robot_name)/robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    '$(arg robot_name)/robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            parameters=[
                {
                    '$(arg robot_name)/robot_description': None
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
