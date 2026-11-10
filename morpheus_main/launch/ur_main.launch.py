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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    # Robot arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # End effector arguments
    ee_type = LaunchConfiguration("ee_type")
    # Control arguments
    controller = LaunchConfiguration("controller").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    # Moveit arguments
    collision = LaunchConfiguration("collision").perform(context)

    # Determine arm and gripper group names
    ee_str = ee_type.perform(context)
    arm_group = "arm" if ee_str else "ur_manipulator"
    gripper_group = "gripper"

    ur_control_node = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("morpheus_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "ee_type": ee_type,
            "use_mock_hardware": use_mock_hardware,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    
    control_nodes = []

    # Teleoperation controller
    if controller in ["gamepad", "ps4"]:
        control_nodes.append(
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("morpheus_teleop"), "launch", "gamepad_twist.launch.py"]
                    )
                ),
                launch_arguments={
                    "controller_type": controller,
                    "use_sim_time": use_sim_time,
                    "arm_group": arm_group,
                    "frame_id": "base_link",
                }.items(),
            )
        )
    elif controller == "xbox":
        control_nodes.append(
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("morpheus_teleop"), "launch", "xbox360_twist.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "arm_group": arm_group,
                    "frame_id": "base_link",
                }.items(),
            )
        )
    elif controller == "keyboard":
        control_nodes.append(
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("morpheus_teleop"), "launch", "key_gamepad_twist.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "arm_group": arm_group,
                    "frame_id": "base_link",
                }.items(),
            )
        )
    elif controller == "leader_follower":
        factr_node = Node(
            package="factr_teleop",
            executable="factr_teleop_ur_ros",
            name="factr_teleop_ur_ros",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time.perform(context).lower() == "true"},
            ],
        )
        control_nodes.append(factr_node)
    if collision.lower() == "true":
        moveit_node = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("morpheus_moveit"), "launch", "ur_moveit.launch.py"]
                )
            ),
            launch_arguments={
                "ur_type": ur_type,
                "robot_ip": robot_ip,
                "ee_type": ee_type,
                "launch_rviz": "false",
                "use_sim_time": use_sim_time,
            }.items(),
        )
        collision_node = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("morpheus_collision"), "launch", "collision.launch.py"]
                )
            ),
            launch_arguments={
                "arm_group": arm_group,
                "gripper_group": gripper_group,
            }.items(),
        )
        control_nodes.append(moveit_node)
        control_nodes.append(collision_node)

    nodes_to_start = [
        ur_control_node,
    ] + control_nodes

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
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
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", 
            default_value="192.168.1.102",
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_type",
            description="Type/series of end effector.",
            choices=[
                "",
                "robotiq",
                "onrobot",
            ],
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            description="Type of control device.",
            choices=[
                "",
                "gamepad",
                "ps4",
                "xbox",
                "keyboard",
                "leader_follower",
            ],
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_id",
            description="Reference frame for control commands.",
            default_value="base_link",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "collision",
            description="Whether or not to use collision detection.",
            default_value="false",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
