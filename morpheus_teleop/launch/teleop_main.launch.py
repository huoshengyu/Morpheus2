import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_model',
            default_value='ur5e'
        ),
        launch.actions.DeclareLaunchArgument(
            name='threshold',
            default_value='0.75'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller_type',
            default_value='xbox360'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gripper_type',
            default_value='robotiq'
        ),
        launch.actions.DeclareLaunchArgument(
            name='twist_topic',
            default_value='target_twist'
        ),
        launch.actions.DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        launch.actions.DeclareLaunchArgument(
            name='frame_id',
            default_value='world'
        ),
        launch.actions.DeclareLaunchArgument(
            name='end_effector',
            default_value='tcp_link'
        ),
        launch_ros.actions.Node(
            package='interbotix_xsarm_joy',
            executable='xsarm_joy',
            name='xsarm_joy',
            output='screen',
            parameters=[
                {
                    'threshold': launch.substitutions.LaunchConfiguration('threshold')
                },
                {
                    'controller': launch.substitutions.LaunchConfiguration('controller_type')
                }
            ],
            condition=launch.conditions.IfCondition(
                "$(eval arg('robot_model') == 'vx300s')")
        ),
        launch_ros.actions.Node(
            package='interbotix_xsarm_joy',
            executable='xsarm_robot',
            name='xsarm_robot',
            output='screen',
            parameters=[
                {
                    'robot_model': launch.substitutions.LaunchConfiguration('robot_model')
                }
            ],
            condition=launch.conditions.IfCondition(
                "$(eval arg('robot_model') == 'vx300s')")
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {
                    'dev': '/dev/input/js0'
                },
                {
                    'dev_ff': '/dev/input/event7'
                },
                {
                    'default_trig_val': 'true'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='morpheus_teleop',
            executable='teleop_main.py',
            name='teleop_main',
            output='screen',
            parameters=[
                {
                    'robot_model': launch.substitutions.LaunchConfiguration('robot_model')
                },
                {
                    'controller_type': launch.substitutions.LaunchConfiguration('controller_type')
                },
                {
                    'gripper_type': launch.substitutions.LaunchConfiguration('gripper_type')
                },
                {
                    'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic')
                },
                {
                    'arm_group': launch.substitutions.LaunchConfiguration('arm_group')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/twist_to_pose.launch.py')
            ),
            launch_arguments={
                'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic'),
                'pose_topic': 'target_frame',
                'wrench_topic': 'target_wrench',
                'frame_id': launch.substitutions.LaunchConfiguration('frame_id'),
                'end_effector': launch.substitutions.LaunchConfiguration('end_effector')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
