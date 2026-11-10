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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_mode = LaunchConfiguration('robot_mode').perform(context)
    control_mode = LaunchConfiguration('control_mode').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    use_keyboard = LaunchConfiguration('use_keyboard').perform(context)

    hardware_type = 'actual' if robot_mode == 'real' else 'fake'

    actions = []

    # 1. Hardware driver
    xsarm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('morpheus_driver'), 'launch', 'xsarm_ros_control.launch.py']
            )
        ),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name,
            'hardware_type': hardware_type,
            'use_rviz': 'false',
        }.items(),
    )
    actions.append(xsarm_driver)

    # 2. Teleoperation controller
    frame_id = f"{robot_name}/base_link"
    end_effector = f"{robot_name}/ee_gripper_link"

    if controller == 'keyboard' or use_keyboard.lower() == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('morpheus_teleop'), 'launch', 'key_gamepad_twist.launch.py']
                    )
                ),
                launch_arguments={
                    'frame_id': frame_id,
                    'arm_group': 'arm',
                    'end_effector': end_effector,
                }.items(),
            )
        )
    elif controller == 'xbox':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('morpheus_teleop'), 'launch', 'xbox360_twist.launch.py']
                    )
                ),
                launch_arguments={
                    'frame_id': frame_id,
                    'arm_group': 'arm',
                    'end_effector': end_effector,
                }.items(),
            )
        )
    elif controller in ['ps4', 'gamepad']:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('morpheus_teleop'), 'launch', 'gamepad_twist.launch.py']
                    )
                ),
                launch_arguments={
                    'controller_type': controller,
                    'frame_id': frame_id,
                    'arm_group': 'arm',
                    'end_effector': end_effector,
                }.items(),
            )
        )

    # 3. Trossen Joy command translator if control is enabled
    if controller not in ['', 'none'] or control_mode not in ['', 'none']:
        trossen_joy_node = Node(
            package='morpheus_teleop',
            executable='trossen_joy.py',
            name='trossen_joy',
            output='screen',
            parameters=[
                {'robot_model': robot_model},
                {'publishing_rate': 50.0},
                {'joy_topic': '/joy'},
                {'pose_topic': 'target_frame'},
                {'wrench_topic': 'target_wrench'},
            ],
        )
        actions.append(trossen_joy_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='vx300s'),
        DeclareLaunchArgument('robot_name', default_value='vx300s'),
        DeclareLaunchArgument('robot_mode', default_value='real'),
        DeclareLaunchArgument('control_mode', default_value='none'),
        DeclareLaunchArgument('controller', default_value='none'),
        DeclareLaunchArgument('use_keyboard', default_value='false'),
        DeclareLaunchArgument('xacro_path', default_value=''),
        DeclareLaunchArgument('srdf_path', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
