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
            default_value=' -J elbow_joint 1.5708 -J finger_joint 0 -J left_outer_knuckle_joint 0 -J shoulder_lift_joint -1.5708 -J shoulder_pan_joint 0 -J wrist_1_joint -1.5708 -J wrist_2_joint -1.5708 -J wrist_3_joint 3.14',
            description='Initial joint configuration of the robot'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controllers',
            default_value='joint_state_controller pos_joint_traj_controller gripper_action_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='stopped_controllers',
            default_value='cartesian_motion_controller motion_control_handle cartesian_force_controller cartesian_compliance_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller_config_file',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/gazebo_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='moveit_controller_config_file',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/b_bot_moveit_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/b_bot_double_sided_shelf_scene.xacro'
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
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_rviz'))
        ),
        launch_ros.actions.Node(
            package='compliant_trajectory_control',
            executable='follow_compliant_trajectory_action_server',
            name='compliant_traj_action_server',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_moveit'), 'launch/common_gazebo.launch.py')
            ),
            launch_arguments={
                'initial_joint_positions': launch.substitutions.LaunchConfiguration('initial_joint_positions'),
                'controllers': launch.substitutions.LaunchConfiguration('controllers'),
                'stopped_controllers': launch.substitutions.LaunchConfiguration('stopped_controllers'),
                'controller_config_file': launch.substitutions.LaunchConfiguration('controller_config_file'),
                'urdf_path': launch.substitutions.LaunchConfiguration('xacro_path')
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
                'load_robot_description': 'true',
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
