import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gazebo_gui',
            default_value='true',
            description='Start Gazebo GUI'
        ),
        launch.actions.DeclareLaunchArgument(
            name='paused',
            default_value='false',
            description='Start Gazebo paused'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_pose',
            default_value='-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0',
            description='Pose to spawn the robot at'
        ),
        launch.actions.DeclareLaunchArgument(
            name='initial_joint_positions',
            default_value=' -J elbow_joint 1.5708 -J finger_joint 0 -J shoulder_lift_joint -1.5708 -J shoulder_pan_joint 0 -J wrist_1_joint -1.5708 -J wrist_2_joint -1.5708 -J wrist_3_joint 3.14',
            description='Initial joint configuration of the robot'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controllers',
            default_value='joint_state_controller arm_traj_controller gripper_action_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='stopped_controllers',
            default_value='arm_pos_controller gripper_pos_controller'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller_config_file',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/a_bot_controllers.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/a_bot_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='unpause',
            default_value="$(eval '' if arg('paused') else '-unpause')"
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_gazebo_model',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='controller_manager',
            executable='spawner',
            name='ros_control_controller_spawner',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='controller_manager',
            executable='spawner',
            name='ros_control_stopped_spawner',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                }
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
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
                    'gazebo_ros'), 'launch/empty_world.launch.py')
            ),
            launch_arguments={
                'paused': 'true',
                'gui': launch.substitutions.LaunchConfiguration('gazebo_gui')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
