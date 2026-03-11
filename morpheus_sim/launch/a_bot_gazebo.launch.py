import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='initial_joint_positions',
            default_value=' -J elbow_joint 1.5708 -J finger_joint 0 -J shoulder_lift_joint -1.5708 -J shoulder_pan_joint 0 -J wrist_1_joint -1.5708 -J wrist_2_joint -1.5708 -J wrist_3_joint 3.14',
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
                'morpheus_moveit') + '/config/ur/ur5e/controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='moveit_controller_config_file',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/ur/ur5e/moveit_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/a_bot_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
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
                'urdf_path': launch.substitutions.LaunchConfiguration('urdf_path')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_moveit'), 'launch/move_group.launch.py')
            ),
            launch_arguments={
                'moveit_controller_manager': 'simple',
                'load_robot_description': 'false'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
