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
                'homestri_gazebo') + '/config/b_bot_moveit_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/a_bot_node2_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/a_bot_scene.srdf'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
