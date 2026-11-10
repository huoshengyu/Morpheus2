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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='world_frame',
            default_value='base_link',
            description='World frame for goal haptics'
        ),
        DeclareLaunchArgument(
            name='ee_frame',
            default_value='tool0',
            description='End effector frame for goal haptics'
        ),
        Node(
            package='morpheus_trajectory',
            executable='goal_haptics.py',
            name='goal_haptics',
            output='screen',
            parameters=[
                {'world_frame': LaunchConfiguration('world_frame')},
                {'ee_frame': LaunchConfiguration('ee_frame')},
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('morpheus_trajectory'), 'launch', 'goal_transform.launch.py']
                )
            )
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
