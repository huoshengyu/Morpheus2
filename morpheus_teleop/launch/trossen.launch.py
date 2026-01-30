import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_model',
            default_value='vx300s'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_name',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_model')
        ),
        launch.actions.DeclareLaunchArgument(
            name='base_link_frame',
            default_value='base_link'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_ar_tag',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_gripper_bar',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='show_gripper_fingers',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_world_frame',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='load_gazebo_configs',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world_name',
            default_value=get_package_share_directory(
                'interbotix_xsarm_gazebo') + '/worlds/xsarm_gazebo.world'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dof',
            default_value='6'
        ),
        launch.actions.DeclareLaunchArgument(
            name='motor_configs',
            default_value=launch.substitutions.LaunchConfiguration(
                'robot_model')
        ),
        launch.actions.DeclareLaunchArgument(
            name='mode_configs',
            default_value=get_package_share_directory(
                'interbotix_xsarm_control') + '/config/modes.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='load_configs',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_mode',
            default_value='fake'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_mode',
            default_value='interbotix'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_moveit_rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz_frame',
            default_value='world'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_cpp_interface',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='moveit_interface_gui',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_python_interface',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xacro_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/urdf/scenes/UCD_trossen_hera_scene.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='srdf_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/srdf/scenes/trossen_hera_scene.srdf.xacro'
        ),
        launch.actions.DeclareLaunchArgument(
            name='joint_limits_path',
            default_value=launch.substitutions.LaunchConfiguration('dof')
        ),
        launch.actions.DeclareLaunchArgument(
            name='cartesian_limits_path',
            default_value=get_package_share_directory(
                'morpheus_moveit') + '/config/cartesian_limits.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='kinematics_path',
            default_value=get_package_share_directory(
                'interbotix_xsarm_moveit') + '/config/kinematics.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='external_urdf_loc',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='external_srdf_loc',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=launch.substitutions.LaunchConfiguration(
                'xacro_path')
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_gazebo',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_actual',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_fake',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_gazebo',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_actual',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_fake',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xs_sdk_type',
            default_value='xs_sdk'
        ),
        launch.actions.DeclareLaunchArgument(
            name='xs_sdk_type',
            default_value='xs_sdk_sim'
        ),
        launch.actions.DeclareLaunchArgument(
            name='threshold',
            default_value='0.75'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='ps4'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_keyboard',
            default_value='false'
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            output='screen',
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_moveit_rviz'))
        ),
        launch_ros.actions.Node(
            package='interbotix_xs_sdk',
            executable=launch.substitutions.LaunchConfiguration('xs_sdk_type'),
            name='xs_sdk',
            output='screen',
            parameters=[
                {
                    'motor_configs': launch.substitutions.LaunchConfiguration('motor_configs')
                },
                {
                    'mode_configs': launch.substitutions.LaunchConfiguration('mode_configs')
                },
                {
                    'load_configs': launch.substitutions.LaunchConfiguration('load_configs')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'interbotix_xsarm_gazebo'), 'launch/xsarm_gazebo.launch.py')
            ),
            launch_arguments={
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'base_link_frame': launch.substitutions.LaunchConfiguration('base_link_frame'),
                'show_ar_tag': launch.substitutions.LaunchConfiguration('show_ar_tag'),
                'use_world_frame': launch.substitutions.LaunchConfiguration('use_world_frame'),
                'external_urdf_loc': launch.substitutions.LaunchConfiguration('external_urdf_loc'),
                'world_name': launch.substitutions.LaunchConfiguration('world_name'),
                'use_trajectory_controllers': 'true'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'interbotix_xsarm_descriptions'), 'launch/xsarm_description.launch.py')
            ),
            launch_arguments={
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'base_link_frame': launch.substitutions.LaunchConfiguration('base_link_frame'),
                'show_ar_tag': launch.substitutions.LaunchConfiguration('show_ar_tag'),
                'use_world_frame': launch.substitutions.LaunchConfiguration('use_world_frame'),
                'external_urdf_loc': launch.substitutions.LaunchConfiguration('external_urdf_loc'),
                'model': launch.substitutions.LaunchConfiguration('model'),
                'use_rviz': 'false',
                'use_joint_pub': 'false',
                'rate': '100',
                'source_list': '[move_group/fake_controller_joint_states]'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_moveit'), 'launch/move_group.launch.py')
            ),
            launch_arguments={
                'load_robot_description': 'false',
                'xacro_path': launch.substitutions.LaunchConfiguration('xacro_path'),
                'srdf_path': launch.substitutions.LaunchConfiguration('srdf_path'),
                'joint_limits_path': launch.substitutions.LaunchConfiguration('joint_limits_path'),
                'cartesian_limits_path': launch.substitutions.LaunchConfiguration('cartesian_limits_path'),
                'kinematics_path': launch.substitutions.LaunchConfiguration('kinematics_path'),
                'publish_monitored_planning_scene': 'true'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/xsarm_joy.launch.py')
            ),
            launch_arguments={
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'robot_name': launch.substitutions.LaunchConfiguration('robot_name'),
                'base_link_frame': launch.substitutions.LaunchConfiguration('base_link_frame'),
                'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz'),
                'mode_configs': launch.substitutions.LaunchConfiguration('mode_configs'),
                'threshold': launch.substitutions.LaunchConfiguration('threshold'),
                'controller': launch.substitutions.LaunchConfiguration('controller'),
                'launch_driver': 'false',
                'use_keyboard': launch.substitutions.LaunchConfiguration('use_keyboard')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/key_to_joy.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
