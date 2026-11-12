# ---------------------------------------------------------------------------
# Morpheus2: ROS2 control and feedback for manual teleoperation research
# Copyright (C) 2026 Brian Huo
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# ---------------------------------------------------------------------------

from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    robot_name_str = robot_name_launch_arg.perform(context)
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    use_mock_hardware_str = LaunchConfiguration('use_mock_hardware').perform(context).lower()
    
    # Determine hardware type: mock hardware overrides hardware_type to 'fake'
    if use_mock_hardware_str == 'true':
        hardware_type_str = 'fake'
    else:
        hardware_type_str = LaunchConfiguration('hardware_type').perform(context)

    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')
    controller_spawner_timeout = LaunchConfiguration('controller_spawner_timeout').perform(context)
    initial_arm_controller = LaunchConfiguration('initial_arm_controller').perform(context)

    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=LaunchConfiguration('hardware_type')
    )

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('morpheus_driver'),
            'config',
            'trossen',
            'vx300s',
            'controllers.yaml',
        ]),
        allow_substs=True
    )

    # 1. Hardware Driver Layer
    xsarm_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'use_rviz': 'false',
            'mode_configs': mode_configs_launch_arg,
            'hardware_type': hardware_type_str,
            'use_sim_time': use_sim_time_param,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='actual'
        ) if use_mock_hardware_str != 'true' else None,
    )

    xsarm_descriptions_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_descriptions'),
                'launch',
                'xsarm_description.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'use_rviz': 'false',
            'mode_configs': mode_configs_launch_arg,
            'hardware_type': hardware_type_str,
            'use_sim_time': use_sim_time_param,
        }.items(),
    )
    
    rviz_node = Node(
        package="rviz2",
        namespace=robot_name_launch_arg,
        condition=IfCondition(use_rviz_launch_arg),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # 2. Controller Manager (ros2_control_node)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_name_launch_arg,
        parameters=[
            {'robot_description': robot_description_launch_arg},
            ros2_control_controllers_config_parameter_file,
        ],
        output={'both': 'screen'},
        arguments=[
            '--ros-args',
            '-r',
            'motion_control_handle/target_frame:=target_frame',
            '-r',
            'cartesian_motion_controller/target_frame:=target_frame',
            '-r',
            'cartesian_compliance_controller/target_frame:=target_frame',
            '-r',
            'cartesian_force_controller/target_wrench:=target_wrench',
            '-r',
            'cartesian_compliance_controller/target_wrench:=target_wrench',
            '-r',
            'cartesian_force_controller/ft_sensor_wrench:=ft_sensor_wrench',
            '-r',
            'cartesian_compliance_controller/ft_sensor_wrench:=ft_sensor_wrench',   
        ]
    )

    # 3. Controller Spawners
    nodes_to_start = [controller_manager_node, rviz_node]

    if hardware_type_str == 'actual':
        nodes_to_start.append(xsarm_control_launch_include)
    else:
        nodes_to_start.append(xsarm_descriptions_launch_include)

    # Always active controllers: joint_state_broadcaster and gripper_controller
    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_str}/controller_manager',
            '--controller-manager-timeout',
            controller_spawner_timeout,
            'joint_state_broadcaster',
        ],
        output={'both': 'screen'},
    )
    nodes_to_start.append(spawn_joint_state_broadcaster_node)

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_str}/controller_manager',
            '--controller-manager-timeout',
            controller_spawner_timeout,
            'gripper_controller',
        ],
        output={'both': 'screen'},
    )
    nodes_to_start.append(spawn_gripper_controller_node)

    # Arm controllers: Cartesian teleop vs Joint trajectory
    # Active controller is started immediately; alternative arm controllers are loaded inactive
    # so MoveIt or teleoperation can switch seamlessly without spawning during runtime
    if initial_arm_controller == 'arm_controller':
        active_arm_ctrl = 'arm_controller'
        inactive_arm_ctrls = [
            'cartesian_motion_controller',
            'cartesian_compliance_controller',
            'cartesian_force_controller',
            'motion_control_handle',
        ]
    else:
        active_arm_ctrl = 'cartesian_motion_controller'
        inactive_arm_ctrls = [
            'arm_controller',
            'cartesian_compliance_controller',
            'cartesian_force_controller',
            'motion_control_handle',
        ]

    spawn_active_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_str}/controller_manager',
            '--controller-manager-timeout',
            controller_spawner_timeout,
            active_arm_ctrl,
        ],
        output={'both': 'screen'},
    )
    nodes_to_start.append(spawn_active_arm_controller_node)

    spawn_inactive_controllers_node = Node(
        name='inactive_controllers_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_str}/controller_manager',
            '--controller-manager-timeout',
            controller_spawner_timeout,
            '--inactive',
        ] + inactive_arm_ctrls,
        output={'both': 'screen'},
    )
    nodes_to_start.append(spawn_inactive_controllers_node)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            default_value='vx300s',
            description=(
                "model type of the Interbotix Arm such as 'wx200' or 'vx300s'."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='use mock hardware driver if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_arm_controller',
            default_value='cartesian_motion_controller',
            choices=('cartesian_motion_controller', 'arm_controller'),
            description='arm controller to activate initially at startup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='10',
            description='timeout for controller spawner service availability in seconds.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("morpheus_description"), "rviz", "xsarm_description.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_ros_control'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get simulation time published over /clock.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            hardware_type='actual'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
