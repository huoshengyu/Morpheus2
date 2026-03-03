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
            name='robot_name',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_model')
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_joint_positions',
            default_value=' -J elbow_joint 1.5708 -J finger_joint 0 -J shoulder_lift_joint -1.5708 -J shoulder_pan_joint 0 -J wrist_1_joint -1.5708 -J wrist_2_joint -1.5708 -J wrist_3_joint 3.14',
            description='Initial joint configuration of the robot'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controllers',
            default_value='joint_state_controller cartesian_compliance_controller gripper_action_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='stopped_controllers',
            default_value='cartesian_motion_controller motion_control_handle cartesian_force_controller pos_joint_traj_controller joint_group_pos_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller_config_file',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/gazebo_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='moveit_controller_config_file',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/a_bot_moveit_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/a_bot_keyhole_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/a_bot_keyhole_scene.srdf'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/a_bot_fake.launch.py')
            ),
            launch_arguments={
                'initial_joint_positions': launch.substitutions.LaunchConfiguration('initial_joint_positions'),
                'controllers': launch.substitutions.LaunchConfiguration('controllers'),
                'stopped_controllers': launch.substitutions.LaunchConfiguration('stopped_controllers'),
                'controller_config_file': launch.substitutions.LaunchConfiguration('controller_config_file'),
                'moveit_controller_config_file': launch.substitutions.LaunchConfiguration('moveit_controller_config_file'),
                'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz'),
                'xacro_path': launch.substitutions.LaunchConfiguration('xacro_path'),
                'srdf_path': launch.substitutions.LaunchConfiguration('srdf_path'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/key_to_joy.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/xbox360_swap_fake.launch.py')
            ),
            launch_arguments={
                'twist_topic': 'twist_controller/command',
                'arm_group': 'arm',
                'frame_id': 'base_link',
                'end_effector': 'tcp_link'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
