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
    args = {
        'namespace': LaunchConfiguration('namespace'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'launch_joy_node': LaunchConfiguration('launch_joy_node'),
        'device_id': LaunchConfiguration('device_id'),
        'device_name': LaunchConfiguration('device_name'),
        'deadzone': LaunchConfiguration('deadzone'),
        'arm_group': LaunchConfiguration('arm_group'),
        'controller_type': LaunchConfiguration('controller_type'),
        'frame_id': LaunchConfiguration('frame_id'),
        'end_effector': LaunchConfiguration('end_effector'),
        'joy_topic': LaunchConfiguration('joy_topic'),
        'twist_topic': LaunchConfiguration('twist_topic'),
        'wrench_topic': LaunchConfiguration('wrench_topic'),
        'pose_topic': LaunchConfiguration('pose_topic'),
        'gripper_topic': LaunchConfiguration('gripper_topic'),
    }

    gamepad_twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('morpheus_teleop'), 'launch', 'gamepad_twist.launch.py']
            )
        ),
        launch_arguments=args.items(),
    )
    return [gamepad_twist]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_joy_node', default_value='true'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('device_name', default_value=''),
        DeclareLaunchArgument('deadzone', default_value='0.05'),
        DeclareLaunchArgument('arm_group', default_value='arm'),
        DeclareLaunchArgument('controller_type', default_value='gamepad'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('end_effector', default_value='tool0'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('twist_topic', default_value='target_twist'),
        DeclareLaunchArgument('wrench_topic', default_value='target_wrench'),
        DeclareLaunchArgument('pose_topic', default_value='target_frame'),
        DeclareLaunchArgument('gripper_topic', default_value='gripper_command'),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
