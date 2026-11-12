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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'base_dir',
            default_value='/root/ros2_ws/data',
            description='Target directory for CSV and rosbag2 output',
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='base_link',
            description='World reference frame for end-effector pose tracking',
        ),
        DeclareLaunchArgument(
            'ee_frame',
            default_value='tool0',
            description='End-effector frame for pose tracking',
        ),
        DeclareLaunchArgument(
            'log_rate_hz',
            default_value='20.0',
            description='Data logging frequency in Hz',
        ),
        DeclareLaunchArgument(
            'record_tf',
            default_value='true',
            description='Whether to log end-effector TF poses to CSV',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Whether to simultaneously record rosbag2',
        ),
        DeclareLaunchArgument(
            'subject_id',
            default_value='subject_01',
            description='Experimental participant / subject identifier',
        ),
        DeclareLaunchArgument(
            'task_name',
            default_value='teleoperation',
            description='Task or condition name',
        ),
        DeclareLaunchArgument(
            'trial',
            default_value='1',
            description='Trial index number',
        ),
    ]

    session_manager_node = Node(
        package='morpheus_data',
        executable='session_manager.py',
        name='session_manager',
        output='screen',
        parameters=[
            {
                'subject_id': LaunchConfiguration('subject_id'),
                'task_name': LaunchConfiguration('task_name'),
                'trial': LaunchConfiguration('trial'),
                'auto_start': True,
                'basename_topic': '/UCDsession/csv_basename',
            }
        ],
    )

    data_recorder_node = Node(
        package='morpheus_data',
        executable='data_recorder.py',
        name='data_recorder',
        output='screen',
        parameters=[
            {
                'base_dir': LaunchConfiguration('base_dir'),
                'world_frame': LaunchConfiguration('world_frame'),
                'ee_frame': LaunchConfiguration('ee_frame'),
                'log_rate_hz': LaunchConfiguration('log_rate_hz'),
                'record_tf': LaunchConfiguration('record_tf'),
                'record_bag': LaunchConfiguration('record_bag'),
                'basename_topic': '/UCDsession/csv_basename',
            }
        ],
    )

    return LaunchDescription(declared_arguments + [session_manager_node, data_recorder_node])


if __name__ == '__main__':
    generate_launch_description()
