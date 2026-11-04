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
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        DeclareLaunchArgument(
            name='gripper_group',
            default_value='gripper'
        ),
        Node(
            package='morpheus_collision',
            executable='collision',
            name='collision',
            output='screen',
            parameters=[
                {
                    'arm_group': LaunchConfiguration('arm_group')
                },
                {
                    'gripper_group': LaunchConfiguration('gripper_group')
                }
            ]
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
