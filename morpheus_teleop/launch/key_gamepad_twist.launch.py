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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_id = LaunchConfiguration('frame_id')
    arm_group = LaunchConfiguration('arm_group')
    end_effector = LaunchConfiguration('end_effector')

    gamepad_twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('morpheus_teleop'), 'launch', 'gamepad_twist.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_joy_node': 'false',
            'frame_id': frame_id,
            'arm_group': arm_group,
            'end_effector': end_effector,
        }.items()
    )

    key_to_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('morpheus_teleop'), 'launch', 'key_to_joy.launch.py']
            )
        )
    )

    return [gamepad_twist, key_to_joy]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            name='frame_id',
            default_value='base_link',
            description='Reference frame id for twist control',
        ),
        DeclareLaunchArgument(
            name='arm_group',
            default_value='arm',
            description='Planning group name',
        ),
        DeclareLaunchArgument(
            name='end_effector',
            default_value='tool0',
            description='End effector frame id',
        ),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    generate_launch_description()
