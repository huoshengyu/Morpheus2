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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_sim_gz_pkg_path = FindPackageShare('ur_simulation_gz')
    ur_sim_control_launch_file = PathJoinSubstitution([ur_sim_gz_pkg_path, 'launch', 'ur_sim_control.launch.py'])
    
    moveit_config_pkg_path = FindPackageShare('morpheus_moveit')
    moveit_launch_file = PathJoinSubstitution([moveit_config_pkg_path, 'launch', 'ur_moveit.launch.py'])
    
    morpheus_sim_pkg_path = FindPackageShare('morpheus_sim')
    controllers_file = PathJoinSubstitution([morpheus_sim_pkg_path, 'config', 'ur_controllers.yaml'])
    
    morpheus_description_pkg_path = FindPackageShare('morpheus_description')
    description_file = PathJoinSubstitution([morpheus_description_pkg_path, 'urdf', 'scenes', 'ur_endeffector.urdf.xacro'])
    rviz_config_file = PathJoinSubstitution([morpheus_description_pkg_path, 'config', 'morpheus_ur5e.rviz'])
    
    initial_joint_controller = 'cartesian_compliance_controller'
    
    ur_type = 'ur5e'

    ur_sim_control_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ur_sim_control_launch_file]),
        launch_arguments=[('ur_type', ur_type),
                          ('controllers_file', controllers_file),
                          ('description_file', description_file),
                          ('rviz_config_file', rviz_config_file),
                          ('initial_joint_controller', initial_joint_controller),
                          ('use_sim_time', 'true'),
                          ('launch_rviz', 'false')],
    )
    
    moveit_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_launch_file]),
        launch_arguments=[('ur_type', ur_type),
                          ('launch_servo', 'true'),
                          ('use_sim_time', 'true'),
                          ('launch_rviz', 'true')],
    )

    # Create the launch description and populate
    ld  = LaunchDescription()
    
    # Add the actions to launch all of the bridge + spawn_model nodes
    ld.add_action(ur_sim_control_description)
    ld.add_action(moveit_description)

    return ld


if __name__ == '__main__':
    generate_launch_description()
