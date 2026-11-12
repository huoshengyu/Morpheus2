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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    robot_desc = LaunchConfiguration('robot_description').perform(context)
    robot_desc_semantic = LaunchConfiguration('robot_description_semantic').perform(context)

    node_params = [
        {'arm_group': LaunchConfiguration('arm_group').perform(context)},
        {'gripper_group': LaunchConfiguration('gripper_group').perform(context)},
        {'robot_description_topic': LaunchConfiguration('robot_description_topic').perform(context)},
        {'joint_states_topic': LaunchConfiguration('joint_states_topic').perform(context)},
        {'planning_scene_topic': LaunchConfiguration('planning_scene_topic').perform(context)},
        {'planning_scene_world_topic': LaunchConfiguration('planning_scene_world_topic').perform(context)},
        {'planning_scene_service': LaunchConfiguration('planning_scene_service').perform(context)},
        {'attached_collision_object_topic': LaunchConfiguration('attached_collision_object_topic').perform(context)},
        {'collision_object_topic': LaunchConfiguration('collision_object_topic').perform(context)},
    ]

    if robot_desc:
        node_params.append({'robot_description': robot_desc})
    if robot_desc_semantic:
        node_params.append({'robot_description_semantic': robot_desc_semantic})

    collision_node = Node(
        package='morpheus_collision',
        executable='collision',
        output='screen',
        parameters=node_params,
    )
    return [collision_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            name='arm_group',
            default_value='arm',
            description='Name of the arm planning group',
        ),
        DeclareLaunchArgument(
            name='gripper_group',
            default_value='gripper',
            description='Name of the gripper planning group',
        ),
        DeclareLaunchArgument(
            name='robot_description_topic',
            default_value='robot_description',
            description='Topic or parameter name for robot URDF description',
        ),
        DeclareLaunchArgument(
            name='joint_states_topic',
            default_value='joint_states',
            description='Topic for joint states',
        ),
        DeclareLaunchArgument(
            name='planning_scene_topic',
            default_value='planning_scene',
            description='Topic for planning scene updates',
        ),
        DeclareLaunchArgument(
            name='planning_scene_world_topic',
            default_value='planning_scene_world',
            description='Topic for planning scene world updates',
        ),
        DeclareLaunchArgument(
            name='planning_scene_service',
            default_value='get_planning_scene',
            description='Service name for get_planning_scene',
        ),
        DeclareLaunchArgument(
            name='attached_collision_object_topic',
            default_value='attached_collision_object',
            description='Topic for attached collision objects',
        ),
        DeclareLaunchArgument(
            name='collision_object_topic',
            default_value='collision_object',
            description='Topic for collision objects',
        ),
        DeclareLaunchArgument(
            name='robot_description',
            default_value='',
            description='Direct URDF XML string (optional override)',
        ),
        DeclareLaunchArgument(
            name='robot_description_semantic',
            default_value='',
            description='Direct SRDF XML string (optional override)',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
