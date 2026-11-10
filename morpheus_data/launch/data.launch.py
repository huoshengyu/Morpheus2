# ---------------------------------------------------------------------------
# Morpheus2: ROS2 control and feedback for manual teleoperation research
# Copyright (C) 2026 Brian Huo
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# ---------------------------------------------------------------------------

import os
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='gamepad'
        ),
        launch.actions.DeclareLaunchArgument(
            name='data',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='base_dir',
            default_value='/root/ros2_ws/data/'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_frame',
            default_value='base_link'
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
                    'base_dir': LaunchConfiguration('base_dir')
                },
                {
                    'basename_topic': '/UCDsession/csv_basename'
                },
                {
                    'world_frame': LaunchConfiguration('world_frame')
                },
                {
                    'log_rate_hz': 10.0
                },
                {
                    'append': False
                },
                {
                    'flush_each_row': False
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
                    'base_dir': LaunchConfiguration('base_dir')
                },
                {
                    'basename_topic': '/UCDsession/csv_basename'
                },
                {
                    'append': False
                },
                {
                    'flush_each_row': False
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
