import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='ps4'
        ),
        launch.actions.DeclareLaunchArgument(
            name='data',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='morpheus_data',
            executable='UCDsession.py',
            name='data_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='morpheus_data',
            executable='trossen_tf_csv',
            name='trossen_tf_csv',
            output='screen',
            parameters=[
                {
                    'base_dir': '/root/catkin_ws/src/morpheus_data/data/'
                },
                {
                    'basename_topic': '/UCDsession/csv_basename'
                },
                {
                    'world_frame': 'vx300s/base_link'
                },
                {
                    'log_rate_hz': '10'
                },
                {
                    'append': 'false'
                },
                {
                    'flush_each_row': 'false'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='morpheus_data',
            executable='Backup_Bag',
            name='Backup_Bag',
            output='screen',
            parameters=[
                {
                    'base_dir': '/root/catkin_ws/src/morpheus_data/data/'
                },
                {
                    'basename_topic': '/UCDsession/csv_basename'
                },
                {
                    'append': 'false'
                },
                {
                    'flush_each_row': 'false'
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
