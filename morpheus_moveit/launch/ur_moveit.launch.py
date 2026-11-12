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
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    """Safely load a YAML configuration file from a ROS package."""
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def declare_arguments():
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz visualization?",
        ),
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        ),
        DeclareLaunchArgument(
            "ee_type",
            description="Type/series of end effector.",
            choices=[
                "",
                "robotiq",
                "onrobot",
            ],
            default_value="",
        ),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Launch MoveIt Servo?",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Using or not time from simulation",
        ),
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
        ),
    ]
    return declared_arguments


def launch_setup(context):
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    ee_type = LaunchConfiguration("ee_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    # 1. Build MoveIt Configurations
    moveit_config_builder = (
        MoveItConfigsBuilder(robot_name="ur", package_name="morpheus_moveit")
    )
    srdf_file = get_package_share_directory("morpheus_description") / Path("srdf") / Path("scenes") / "ur.srdf.xacro"
    if ee_type.perform(context):
        srdf_file = get_package_share_directory("morpheus_description") / Path("srdf") / Path("scenes") / Path("ur_" + ee_type.perform(context) + ".srdf.xacro")
    moveit_config_builder.robot_description_semantic(srdf_file, {"name": ur_type})
    moveit_config_builder.robot_description_kinematics(Path("config") / ur_type.perform(context) / "kinematics.yaml")
    moveit_config_builder.joint_limits(Path("config") / ur_type.perform(context) / "joint_limits.yaml")
    moveit_config_builder.pilz_cartesian_limits(Path("config") / ur_type.perform(context) / "pilz_cartesian_limits.yaml")
    moveit_config_builder.trajectory_execution(Path("config") / ur_type.perform(context) / "moveit_controllers.yaml")
    moveit_config_builder.sensors_3d(Path("config") / ur_type.perform(context) / "sensors_3d.yaml")

    moveit_config = moveit_config_builder.to_moveit_configs()

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # 2. Synchronize on robot description publication
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    # 3. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    # 4. MoveIt Servo Node (optional)
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
    )

    # 5. MoveIt RViz Node (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("morpheus_description"), "rviz", "morpheus_ur5e.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # MoveGroup, RViz, and Servo are started after robot description is published
    wait_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_robot_description,
            on_exit=[move_group_node, rviz_node, servo_node],
        )
    )

    return [
        wait_robot_description,
        wait_event_handler,
    ]


def generate_launch_description():
    ld = LaunchDescription(declare_arguments())
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
