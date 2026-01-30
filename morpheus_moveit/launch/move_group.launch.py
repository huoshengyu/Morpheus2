import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='launch_prefix',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='launch_prefix',
            default_value='gdb -x $(dirname)/gdb_settings.gdb --ex run --args'
        ),
        launch.actions.DeclareLaunchArgument(
            name='info',
            default_value=launch.substitutions.LaunchConfiguration('debug')
        ),
        launch.actions.DeclareLaunchArgument(
            name='command_args',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='command_args',
            default_value='--debug'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pipeline',
            default_value='ompl'
        ),
        launch.actions.DeclareLaunchArgument(
            name='allow_trajectory_execution',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='moveit_controller_manager',
            default_value='simple'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fake_execution_type',
            default_value='interpolate'
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_safe_path_cost',
            default_value='1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='publish_monitored_planning_scene',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='capabilities',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='disable_capabilities',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='load_robot_description',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/test_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/test_scene.srdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='joint_limits_path',
            default_value=get_package_share_directory(
                'homestri_a_bot_moveit_config') + '/config/joint_limits.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cartesian_limits_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/cartesian_limits.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='kinematics_path',
            default_value=get_package_share_directory(
                'homestri_a_bot_moveit_config') + '/config/kinematics.yaml'
        ),
        launch_ros.actions.Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {
                    'allow_trajectory_execution': launch.substitutions.LaunchConfiguration('allow_trajectory_execution')
                },
                {
                    'sense_for_plan/max_safe_path_cost': launch.substitutions.LaunchConfiguration('max_safe_path_cost')
                },
                {
                    'default_planning_pipeline': launch.substitutions.LaunchConfiguration('pipeline')
                },
                {
                    'capabilities': launch.substitutions.LaunchConfiguration('capabilities')
                },
                {
                    'disable_capabilities': launch.substitutions.LaunchConfiguration('disable_capabilities')
                },
                {
                    'monitor_dynamics': 'false'
                },
                {
                    'planning_scene_monitor/publish_planning_scene': launch.substitutions.LaunchConfiguration('publish_monitored_planning_scene')
                },
                {
                    'planning_scene_monitor/publish_geometry_updates': launch.substitutions.LaunchConfiguration('publish_monitored_planning_scene')
                },
                {
                    'planning_scene_monitor/publish_state_updates': launch.substitutions.LaunchConfiguration('publish_monitored_planning_scene')
                },
                {
                    'planning_scene_monitor/publish_transforms_updates': launch.substitutions.LaunchConfiguration('publish_monitored_planning_scene')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'homestri_a_bot_moveit_config'), 'launch/trajectory_execution.launch.xml.py')
            ),
            launch_arguments={
                'moveit_manage_controllers': 'true',
                'moveit_controller_manager': launch.substitutions.LaunchConfiguration('moveit_controller_manager'),
                'fake_execution_type': launch.substitutions.LaunchConfiguration('fake_execution_type')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'homestri_a_bot_moveit_config'), 'launch/sensor_manager.launch.xml.py')
            ),
            launch_arguments={
                'moveit_sensor_manager': 'a_bot_scene'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
