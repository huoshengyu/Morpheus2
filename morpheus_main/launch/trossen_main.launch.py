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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    # Robot arguments
    robot_model = LaunchConfiguration('robot_model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Control arguments
    controller = LaunchConfiguration('controller').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    end_effector = LaunchConfiguration('end_effector').perform(context)
    
    # MoveIt & Collision arguments
    collision = LaunchConfiguration('collision').perform(context).lower() == 'true'
    collision_haptics = LaunchConfiguration('collision_haptics').perform(context).lower() == 'true'
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)

    # Experiment & Recording arguments
    spawner = LaunchConfiguration('spawner').perform(context).lower() == 'true'
    data = LaunchConfiguration('data').perform(context).lower() == 'true'

    hardware_type = 'fake' if use_mock_hardware else 'actual'

    # 1. Driver & Hardware Control (Controller Manager, Spawners, Driver)
    xsarm_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('morpheus_driver'),
                'launch',
                'xsarm_control.launch.py',
            ])
        ),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name,
            'use_mock_hardware': 'true' if use_mock_hardware else 'false',
            'hardware_type': hardware_type,
            'use_sim_time': use_sim_time,
            'use_rviz': launch_rviz,
        }.items(),
    )

    control_nodes = []

    # 2. Teleoperation Controller
    if controller == 'keyboard':
        control_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('morpheus_teleop'),
                        'launch',
                        'key_gamepad_twist.launch.py',
                    ])
                ),
                launch_arguments={
                    'namespace': robot_name,
                    'frame_id': frame_id,
                    'arm_group': 'interbotix_arm',
                    'end_effector': end_effector,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        )
    elif controller in ['gamepad', 'ps4', 'xbox360']:
        control_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('morpheus_teleop'),
                        'launch',
                        'gamepad_twist.launch.py',
                    ])
                ),
                launch_arguments={
                    'namespace': robot_name,
                    'controller_type': 'gamepad',
                    'frame_id': frame_id,
                    'arm_group': 'interbotix_arm',
                    'end_effector': end_effector,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        )
    elif controller in ['leader_follower', 'factr']:
        factr_vx300s_node = Node(
            package='factr_teleop',
            namespace=robot_name,
            executable='factr_teleop_ur_ros',
            name='factr_teleop_vx300s',
            output='screen',
            parameters=[
                {'use_mock_hardware': use_mock_hardware},
                {'config_file': 'vx300s.yaml'},
                {'use_sim_time': use_sim_time.perform(context).lower() == 'true'},
            ],
        )
        control_nodes.append(factr_vx300s_node)

    # 3. MoveIt & Collision Avoidance Pipeline
    if collision:
        # MoveGroup and MoveIt execution pipeline
        moveit_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('morpheus_moveit'),
                    'launch',
                    'xsarm_moveit.launch.py',
                ])
            ),
            launch_arguments={
                'robot_model': robot_model,
                'robot_name': robot_name,
                'use_mock_hardware': 'true' if use_mock_hardware else 'false',
                'hardware_type': hardware_type,
                'launch_rviz': 'false',
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': 'true',
            }.items(),
        )

        # Generate URDF and SRDF for Morpheus HYBRID distance-field collision detector
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
        except Exception:
            urdf_xml = ''

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
        except Exception:
            srdf_xml = ''

        collision_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('morpheus_collision'),
                    'launch',
                    'collision.launch.py',
                ])
            ),
            launch_arguments={
                'arm_group': 'interbotix_arm',
                'gripper_group': 'interbotix_gripper',
                'robot_description': urdf_xml,
                'robot_description_semantic': srdf_xml,
                'robot_description_topic': 'robot_description',
                'joint_states_topic': f'{robot_name}/joint_states',
                'planning_scene_topic': f'{robot_name}/monitored_planning_scene',
                'planning_scene_world_topic': f'{robot_name}/planning_scene_world',
                'planning_scene_service': f'{robot_name}/get_planning_scene',
            }.items(),
        )
        control_nodes.append(moveit_node)
        control_nodes.append(collision_node)

    # 4. Spawner (Experiment objects)
    if spawner:
        spawner_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('morpheus_spawner'),
                    'launch',
                    'spawner.launch.py',
                ])
            )
        )
        control_nodes.append(spawner_node)

    # 5. Haptic Feedback (if requested)
    if collision_haptics:
        haptics_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('morpheus_haptics'),
                    'launch',
                    'collision_send.launch.py',
                ])
            ),
            launch_arguments={
                'mock_mode': 'true' if use_mock_hardware else 'false',
                'arm_type': robot_model,
            }.items(),
        )
        control_nodes.append(haptics_node)

    # 6. Data Recording (if requested)
    if data:
        data_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('morpheus_data'),
                    'launch',
                    'data.launch.py',
                ])
            )
        )
        control_nodes.append(data_node)

    nodes_to_start = [
        xsarm_control_node,
    ] + control_nodes

    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        # Robot arguments
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
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        # Control arguments
        DeclareLaunchArgument(
            'controller',
            default_value='gamepad',
            choices=['gamepad', 'ps4', 'keyboard', 'xbox', 'leader_follower', 'factr', 'none'],
            description='Teleoperation controller interface',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='vx300s/base_link',
            description='Reference frame for control commands',
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='vx300s/ee_gripper_link',
            description='End effector link frame name',
        ),
        # MoveIt & Collision arguments
        DeclareLaunchArgument(
            'collision',
            default_value='false',
            description='Launch MoveIt and Morpheus collision pipeline',
        ),
        DeclareLaunchArgument(
            'collision_haptics',
            default_value='false',
            description='Launch haptic serial feedback',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz visualization',
        ),
        # Experiment & Logging arguments
        DeclareLaunchArgument(
            'spawner',
            default_value='false',
            description='Launch cylinder spawner for collision experiment',
        ),
        DeclareLaunchArgument(
            'data',
            default_value='false',
            description='Launch session manager and data recorder',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
