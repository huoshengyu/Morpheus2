import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_ip',
            default_value='192.168.1.103'
        ),
        launch.actions.DeclareLaunchArgument(
            name='reverse_ip',
            default_value='192.168.1.2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='kinematics_config',
            default_value=get_package_share_directory(
                'homestri_bringup') + '/etc/b_bot_calibration.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controllers',
            default_value='joint_state_controller twist_controller speed_scaling_state_controller force_torque_sensor_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='stopped_controllers',
            default_value='joint_group_vel_controller scaled_pos_joint_traj_controller motion_control_handle cartesian_motion_controller cartesian_force_controller cartesian_compliance_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller_config_file',
            default_value=get_package_share_directory(
                'homestri_bringup') + '/config/b_bot_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='moveit_controller_config_file',
            default_value=get_package_share_directory(
                'homestri_bringup') + '/config/ur/ur5e/moveit_controllers.yaml.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gripper_ip',
            default_value='192.168.1.1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gripper_port',
            default_value='502'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/b_bot_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/b_bot_scene.srdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='twist_topic',
            default_value='twist_controller/command'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/b_bot_bringup.launch.py')
            ),
            launch_arguments={
                'robot_ip': launch.substitutions.LaunchConfiguration('robot_ip'),
                'reverse_ip': launch.substitutions.LaunchConfiguration('reverse_ip'),
                'kinematics_config': launch.substitutions.LaunchConfiguration('kinematics_config'),
                'controllers': launch.substitutions.LaunchConfiguration('controllers'),
                'stopped_controllers': launch.substitutions.LaunchConfiguration('stopped_controllers'),
                'controller_config_file': launch.substitutions.LaunchConfiguration('controller_config_file'),
                'moveit_controller_config_file': launch.substitutions.LaunchConfiguration('moveit_controller_config_file'),
                'gripper_ip': launch.substitutions.LaunchConfiguration('gripper_ip'),
                'gripper_port': launch.substitutions.LaunchConfiguration('gripper_port'),
                'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz'),
                'xacro_path': launch.substitutions.LaunchConfiguration('xacro_path'),
                'srdf_path': launch.substitutions.LaunchConfiguration('srdf_path')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/xbox360_swap.launch.py')
            ),
            launch_arguments={
                'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic'),
                'arm_group': 'arm',
                'frame_id': 'base_link',
                'end_effector': 'tcp_link'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
