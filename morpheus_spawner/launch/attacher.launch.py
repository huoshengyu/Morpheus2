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

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='mesh_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/meshes/collision/dragon_simple.obj'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_pose',
            default_value='-x -0.1 -y 0.8528 -z 0.91915 -R 0.0 -P 0.0 -Y 0.0',
            description='Pose to spawn the robot at'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mode',
            default_value='attach'
        ),
        launch.actions.DeclareLaunchArgument(
            name='index',
            default_value='0'
        ),
        launch_ros.actions.Node(
            package='morpheus_spawner',
            executable='morpheus_spawner',
            name='morpheus_spawner',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
