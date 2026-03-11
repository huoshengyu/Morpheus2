import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_ip',
            default_value='192.168.1.102'
        ),
        launch.actions.DeclareLaunchArgument(
            name='reverse_ip',
            default_value='192.168.1.103'
        ),
        launch.actions.DeclareLaunchArgument(
            name='calibration_path',
            default_value=get_package_share_directory(
                'homestri_bringup') + '/etc/b_bot_calibration.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controllers',
            default_value='joint_state_controller twist_controller speed_scaling_state_controller force_torque_sensor_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='stopped_controllers',
            default_value='scaled_pos_joint_traj_controller joint_group_vel_controller motion_control_handle cartesian_motion_controller cartesian_force_controller cartesian_compliance_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller_config_file',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/ur/ur5e/controllers.yaml'
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
                'morpheus_description') + '/urdf/scenes/b_bot_node2_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/b_bot_scene.srdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='joint_limits_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/joint_limits.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cartesian_limits_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/cartesian_limits.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='kinematics_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/kinematics.yaml'
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_rviz'))
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='compliant_trajectory_control',
            executable='follow_compliant_trajectory_action_server',
            name='compliant_traj_action_server',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ur_robot_driver'), 'launch/ur_control.launch.py')
            ),
            launch_arguments={
                'robot_ip': launch.substitutions.LaunchConfiguration('robot_ip'),
                'reverse_ip': launch.substitutions.LaunchConfiguration('reverse_ip'),
                'use_tool_communication': 'false',
                'kinematics_config': launch.substitutions.LaunchConfiguration('calibration_path'),
                'controllers': launch.substitutions.LaunchConfiguration('controllers'),
                'stopped_controllers': launch.substitutions.LaunchConfiguration('stopped_controllers'),
                'controller_config_file': launch.substitutions.LaunchConfiguration('controller_config_file')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'onrobot_rg2ft_action_server'), 'launch/onrobot_rg2ft_action_server.launch.py')
            ),
            launch_arguments={
                'ip': launch.substitutions.LaunchConfiguration('gripper_ip'),
                'port': launch.substitutions.LaunchConfiguration('gripper_port'),
                'gripper_name': 'onrobot_rg2ft'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_moveit'), 'launch/move_group.launch.py')
            ),
            launch_arguments={
                'allow_trajectory_execution': 'true',
                'moveit_controller_manager': 'simple',
                'fake_execution_type': 'interpolate',
                'info': 'true',
                'debug': 'false',
                'pipeline': 'ompl',
                'load_robot_description': 'false',
                'xacro_path': launch.substitutions.LaunchConfiguration('xacro_path'),
                'srdf_path': launch.substitutions.LaunchConfiguration('srdf_path'),
                'joint_limits_path': launch.substitutions.LaunchConfiguration('joint_limits_path'),
                'cartesian_limits_path': launch.substitutions.LaunchConfiguration('cartesian_limits_path'),
                'kinematics_path': launch.substitutions.LaunchConfiguration('kinematics_path')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
