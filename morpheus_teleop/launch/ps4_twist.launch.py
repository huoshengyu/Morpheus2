import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    # Joy Node Arguments
    dev = LaunchConfiguration("dev")
    dev_ff = LaunchConfiguration("dev_ff")
    default_trig_val = LaunchConfiguration("default_trig_val")
    # Teleop Arguments
    twist_topic = LaunchConfiguration("twist_topic")
    arm_group = LaunchConfiguration("arm_group")
    controller_type = LaunchConfiguration("controller_type")
    gripper_type = LaunchConfiguration("gripper_type")
    robot_model = LaunchConfiguration("robot_model")
    # Twist to Pose Arguments
    frame_id = LaunchConfiguration("frame_id")
    end_effector = LaunchConfiguration("end_effector")
    publishing_rate = LaunchConfiguration("publishing_rate")
    wrench_topic = LaunchConfiguration("wrench_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'dev': dev},
            {'dev_ff': dev_ff},
            {'default_trig_val': default_trig_val},
        ]
    )
    
    teleop_twist_node = Node(
        package='morpheus_teleop',
        executable='teleop_twist.py',
        name='teleop_twist',
        output='screen',
        parameters=[
            {'use_sim_time': True,},
            {'twist_topic': twist_topic,},
            {'arm_group': arm_group,},
            {'controller_type': controller_type,},
            {'gripper_type': gripper_type,},
            {'robot_model': robot_model,},
        ]
    )
    
    twist_to_pose_node = Node(
        package='morpheus_teleop',
        executable='twist_to_pose.py',
        name='twist_to_pose',
        output='screen',
        parameters=[
            {'use_sim_time': True,},
            {'twist_topic': twist_topic,},
            {'frame_id': frame_id,},
            {'end_effector': end_effector,},
            {'publishing_rate': publishing_rate,},
            {'wrench_topic': wrench_topic,},
            {'pose_topic': pose_topic,},
        ]
    )
    
    nodes_to_start = [
        joy_node,
        teleop_twist_node,
        twist_to_pose_node,
    ]
    
    return nodes_to_start
    
    
def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name='dev',
            default_value='/dev/input/js0',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='dev_ff',
            default_value='/dev/input/event7',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='default_trig_val',
            default_value='true',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='twist_topic',
            default_value='twist_controller/command',
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
            default_value='ps4',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='gripper_type',
            default_value='gazebo',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='robot_model',
            default_value='ur5e',
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
            name='publishing_rate',
            default_value='125',
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
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
