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
    OpaqueFunction,
    GroupAction,
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node, PushROSNamespace


def launch_setup(context):
    # Group arguments
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_joy_node = LaunchConfiguration("launch_joy_node")
    # Joy Node Arguments
    device_id = LaunchConfiguration("device_id")
    device_name = LaunchConfiguration("device_name")
    deadzone = LaunchConfiguration("deadzone")
    # Teleop Arguments
    arm_group = LaunchConfiguration("arm_group")
    controller_type = LaunchConfiguration("controller_type")
    # Twist to Pose Arguments
    frame_id = LaunchConfiguration("frame_id")
    end_effector = LaunchConfiguration("end_effector")
    joy_topic = LaunchConfiguration("joy_topic")
    twist_topic = LaunchConfiguration("twist_topic")
    wrench_topic = LaunchConfiguration("wrench_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    gripper_topic = LaunchConfiguration("gripper_topic")
    
    actions = [PushROSNamespace(namespace)]

    if launch_joy_node.perform(context).lower() == 'true':
        joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'device_id': device_id},
                {'device_name': device_name},
                {'deadzone': deadzone},
                {'autorepeat_rate': 20.0},
                {'sticky_buttons': False},
                {'coalesce_interval_ms': 1},
            ]
        )
        actions.append(joy_node)
    
    teleop_twist_node = Node(
        package='morpheus_teleop',
        executable='teleop_twist.py',
        name='teleop_twist',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time.perform(context).lower() == 'true'},
            {'arm_group': arm_group},
            {'controller_type': controller_type},
            {'frame_id': frame_id},
            {'end_effector': end_effector},
            {'joy_topic': joy_topic},
            {'twist_topic': twist_topic},
            {'wrench_topic': wrench_topic},
            {'pose_topic': pose_topic},
            {'gripper_topic': gripper_topic},
        ]
    )
    actions.append(teleop_twist_node)
    
    nodes_to_start = [GroupAction(actions=actions)]
    return nodes_to_start
    
    
def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name='namespace',
            default_value='',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='launch_joy_node',
            default_value='true',
            description='Whether to launch the joy_node driver for a physical gamepad',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='device_id',
            default_value='0',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='device_name',
            default_value='',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='deadzone',
            default_value='0.05',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='arm_group',
            default_value='arm',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='controller_type',
            default_value='gamepad',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='frame_id',
            default_value='base_link',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='end_effector',
            default_value='tool0',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='joy_topic',
            default_value='joy',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='twist_topic',
            default_value='target_twist',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='wrench_topic',
            default_value='target_wrench',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='pose_topic',
            default_value='target_frame',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='gripper_topic',
            default_value='gripper_command',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
