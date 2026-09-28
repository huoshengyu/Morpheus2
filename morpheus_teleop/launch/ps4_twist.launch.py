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
    
    teleop_twist_node = Node(
        package='morpheus_teleop',
        executable='teleop_twist.py',
        name='teleop_twist',
        output='screen',
        parameters=[
            {'use_sim_time': True,},
            {'arm_group': arm_group,},
            {'controller_type': controller_type,},
            {'frame_id': frame_id,},
            {'end_effector': end_effector,},
            {'joy_topic': joy_topic,},
            {'twist_topic': twist_topic,},
            {'wrench_topic': wrench_topic,},
            {'pose_topic': pose_topic,},
            {'gripper_topic': gripper_topic,},
        ]
    )
    
    nodes_to_start = [
        GroupAction(
            actions=[
                PushROSNamespace(namespace),
                joy_node,
                teleop_twist_node,
            ]
        )
    ]
    
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
            default_value='ps4',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='frame_id',
            default_value='base',
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
            default_value='/joy',
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
