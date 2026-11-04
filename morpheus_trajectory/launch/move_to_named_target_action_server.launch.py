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
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


def launch_setup(context):
    # Semantic Description Arguments
    arm_group = LaunchConfiguration("arm_group")
    gripper_group = LaunchConfiguration("gripper_group")
    trajectory_controller = LaunchConfiguration("trajectory_controller")
    
    move_to_named_target_action_server = Node(
        package='morpheus_trajectory',
        executable='move_to_named_target_action_server',
        name='move_to_named_target_action_server',
        output='screen',
        parameters=[
            {'arm_group': arm_group,},
            {'gripper_group': gripper_group,},
            {'trajectory_controller': trajectory_controller,},
        ]
    )
    
    nodes_to_start = [
        move_to_named_target_action_server,
    ]
    
    return nodes_to_start
    
    
def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name='arm_group',
            default_value='arm',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='gripper_group',
            default_value='gripper',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='trajectory_controller',
            default_value='scaled_joint_trajectory_controller',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
