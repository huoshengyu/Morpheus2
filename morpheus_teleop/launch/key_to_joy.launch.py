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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    
    key_to_joy = Node(
            package='morpheus_teleop',
            namespace=namespace,
            executable='key_to_joy.py',
            name='key_to_joy',
            output='screen'
        )
    
    return [key_to_joy]
    
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            name='namespace',
            default_value='',
        ),
    ]
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


if __name__ == '__main__':
    generate_launch_description()
