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
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='trajectory_name',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='goal_name_vector',
            default_value='trossen_home hera_ceiling hera_floor hera_crouch_front hera_crouch_right hera_arch_over hera_arch_pick'
        ),
        launch.actions.DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gripper_group',
            default_value='gripper'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mode',
            default_value='preset'
        ),
        launch.actions.DeclareLaunchArgument(
            name='controller',
            default_value='ps4'
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory_haptics',
            default_value='false'
        ),
        launch_ros.actions.Node(
            package='morpheus_trajectory',
            executable='serial_trajec_sender',
            name='serial_trajec_sender',
            output='screen',
            parameters=[
                {
                    'goal/name_vector': launch.substitutions.LaunchConfiguration('goal_name_vector')
                },
                {
                    'goal/trajectory_name': launch.substitutions.LaunchConfiguration('trajectory_name')
                }
            ],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('trajectory_haptics'))
        ),
        launch_ros.actions.Node(
            package='morpheus_trajectory',
            executable='morpheus_trajectory',
            name='trajectory',
            output='screen',
            parameters=[
                {
                    'goal/name_vector': launch.substitutions.LaunchConfiguration('goal_name_vector')
                },
                {
                    'goal/trajectory_name': launch.substitutions.LaunchConfiguration('trajectory_name')
                },
                {
                    'arm_group': launch.substitutions.LaunchConfiguration('arm_group')
                },
                {
                    'gripper_group': launch.substitutions.LaunchConfiguration('gripper_group')
                },
                {
                    'mode': launch.substitutions.LaunchConfiguration('mode')
                },
                {
                    'controller': launch.substitutions.LaunchConfiguration('controller')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
