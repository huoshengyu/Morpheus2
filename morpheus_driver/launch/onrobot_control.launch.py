# Copyright (c) 2022 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Launch configuration variables
    onrobot_type = LaunchConfiguration('onrobot_type')
    connection_type = LaunchConfiguration('connection_type')
    device = LaunchConfiguration('device')
    ip_address = LaunchConfiguration('ip_address')
    port = LaunchConfiguration('port')
    prefix = LaunchConfiguration('prefix')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rsp = LaunchConfiguration('launch_rsp')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
    description_pkg_share = FindPackageShare(
        package="onrobot_description"
    ).find("onrobot_description")
    default_model_path = os.path.join(
        description_pkg_share, "urdf", "onrobot_macro.xacro"
    )
    default_rviz_config_path = os.path.join(
        description_pkg_share, "rviz", "view_onrobot.rviz"
    )

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description="Absolute path to gripper URDF file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'onrobot_type',
            choices=['rg2', 'rg6'],
            default_value='rg2',
            description='Type of OnRobot gripper.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'connection_type',
            choices=['serial', 'tcp'],
            default_value='tcp',
            description='Connection type for the OnRobot gripper. TCP for the Control Box. Serial for the UR Tool I/O (RS485).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'device',
            default_value='/tmp/ttyUR',
            description='Device name for the serial connection. Only used when connection_type is serial.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.1.1',
            description='IP address for the TCP connection. Only used when connection_type is tcp.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'port',
            default_value='502',
            description='Port for the TCP connection. Only used when connection_type is tcp.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for joint names (useful for multi-robot setups).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="rviz_config_file",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz for visualization.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rsp',
            default_value='true',
            description='Launch robot state publisher.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware interface for testing.',
        )
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        LaunchConfiguration("model"),
        ' ',
        'onrobot_type:=', onrobot_type,
        ' ',
        'connection_type:=', connection_type,
        ' ',
        'device:=', device,
        ' ',
        'ip_address:=', ip_address,
        ' ',
        'port:=', port,
        ' ',
        'prefix:=', prefix,
        ' ',
        'use_fake_hardware:=', use_fake_hardware,
        ' ',
        'name:=onrobot'   
    ])

    robot_description_param = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    controllers_file = "onrobot_controllers.yaml"
    initial_joint_controllers = PathJoinSubstitution(
        [description_pkg_share, "config", controllers_file]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            initial_joint_controllers,
        ],
    )

    # Launch the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        condition=IfCondition(launch_rsp),
        executable='robot_state_publisher',
        parameters=[robot_description_param],
        output='both'
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
    )

    onrobot_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["onrobot_gripper_controller"],
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        onrobot_gripper_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
