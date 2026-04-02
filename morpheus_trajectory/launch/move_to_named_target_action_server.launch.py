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
