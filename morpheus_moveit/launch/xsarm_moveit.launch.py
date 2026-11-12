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

import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    """Safely load a YAML configuration file from a ROS package."""
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return None


def declare_arguments():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='vx300s',
            description='Model of the Interbotix robot arm (vx300s, wx250s, etc.)',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description='Name / namespace of the robot',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware driver if true',
        ),
        DeclareLaunchArgument(
            'hardware_type',
            default_value='actual',
            choices=('actual', 'fake', 'gz_classic'),
            description='Hardware type configuration for driver / simulation',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Launch RViz with MoveIt configuration',
        ),
        DeclareLaunchArgument(
            'rviz_frame',
            default_value='world',
            description='Fixed frame for RViz visualization',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'rviz',
                'xsarm_moveit.rviz',
            ]),
            description='Path to the RViz configuration file',
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='MoveGroup publishes robot description semantic topic',
        ),
        DeclareLaunchArgument(
            'base_link_frame',
            default_value='base_link',
            description='Base link frame name',
        ),
        DeclareLaunchArgument(
            'use_world_frame',
            default_value='true',
            description='Attach base_link to world frame',
        ),
        DeclareLaunchArgument(
            'external_urdf_loc',
            default_value='',
            description='Path to custom URDF if any',
        ),
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value='',
            description='Path to custom SRDF if any',
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value='',
            description='Optional direct URDF string override',
        ),
        DeclareLaunchArgument(
            'robot_description_semantic',
            default_value='',
            description='Optional direct SRDF string override',
        ),
    ]
    return declared_arguments


def launch_setup(context):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    hardware_type = 'fake' if use_mock_hardware else LaunchConfiguration('hardware_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_frame = LaunchConfiguration('rviz_frame')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path').perform(context)
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    # 1. URDF Generation
    robot_desc_override = LaunchConfiguration('robot_description').perform(context)
    if robot_desc_override:
        urdf_xml = robot_desc_override
    else:
        try:
            urdf_xacro_path = os.path.join(
                get_package_share_directory('interbotix_xsarm_descriptions'),
                'urdf',
                f'{robot_model}.urdf.xacro',
            )
            urdf_xml = subprocess.check_output([
                'xacro',
                urdf_xacro_path,
                f'robot_name:={robot_name}',
                f'hardware_type:={hardware_type}',
            ], text=True)
        except Exception as e:
            urdf_xml = ''

    # 2. SRDF Generation
    robot_desc_sem_override = LaunchConfiguration('robot_description_semantic').perform(context)
    if robot_desc_sem_override:
        srdf_xml = robot_desc_sem_override
    else:
        try:
            srdf_xacro_path = os.path.join(
                get_package_share_directory('interbotix_xsarm_moveit'),
                'config',
                'srdf',
                f'{robot_model}.srdf.xacro',
            )
            srdf_xml = subprocess.check_output([
                'xacro',
                srdf_xacro_path,
                f'robot_name:={robot_name}',
            ], text=True)
        except Exception as e:
            srdf_xml = ''

    robot_description = {'robot_description': urdf_xml}
    robot_description_semantic = {'robot_description_semantic': srdf_xml}

    # 3. Kinematics Configuration (KDL Kinematics Plugin)
    kinematics_config = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
        'kinematics.yaml',
    ])

    # 4. OMPL Planning Pipeline (ROS 2 Jazzy format)
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': [
                'ompl_interface/OMPLPlanner',
            ],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        },
    }

    ompl_planning_pipeline_yaml = load_yaml('interbotix_xsarm_moveit', 'config/ompl_planning.yaml')
    if ompl_planning_pipeline_yaml:
        ompl_planning_pipeline_config['ompl'].update(ompl_planning_pipeline_yaml)

    # 5. MoveIt Controllers & Trajectory Execution
    controllers_config = load_yaml(
        'interbotix_xsarm_moveit',
        f'config/controllers/{robot_model}_controllers.yaml',
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_config,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # 6. Planning Scene Monitor Parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # 7. Joint Limits
    config_joint_limits = load_yaml(
        'interbotix_xsarm_moveit',
        f'config/joint_limits/{robot_model}_joint_limits.yaml',
    )
    joint_limits = {'robot_description_planning': config_joint_limits} if config_joint_limits else {}

    # 8. Warehouse Database Configuration
    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    # Remappings for namespaced controller topics
    remappings = [
        (
            '/arm_controller/follow_joint_trajectory',
            f'/{robot_name}/arm_controller/follow_joint_trajectory',
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            f'/{robot_name}/gripper_controller/follow_joint_trajectory',
        ),
        (
            f'/{robot_name}/get_planning_scene',
            f'/{robot_name}/get_planning_scene',
        ),
    ]

    # MoveGroup Node (Pure MoveIt execution, no controller spawners)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': f'/{robot_name}/joint_states',
                },
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
            warehouse_ros_config,
        ],
        remappings=remappings,
        output={'both': 'screen'},
    )

    # Optional MoveIt RViz Node
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        arguments=[
            '-d', rviz_config_file,
            '-f', rviz_frame,
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=remappings,
        output={'both': 'log'},
    )

    return [
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    ld = LaunchDescription(declare_arguments())
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
