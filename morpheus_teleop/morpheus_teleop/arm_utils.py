#! /usr/bin/env python3
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


# General Imports
import numpy as np  

def get_mean_buffer(buffer_list):
    # Get mean of list of buffers
    mean_buffer = [np.mean(np.array(buffer)) for buffer in buffer_list]
    return mean_buffer

def get_arm_target(robot, pose_diff):
    # Get target pose for Trossen robot arm by adding a difference to current end-effector pose
    robot_pose = robot.arm.get_ee_pose()
    pose_target = [pos + diff for pos, diff in zip(robot_pose, pose_diff)]
    return pose_target

def command_trossen_arm(robot, pose_target):
    # Command Trossen robot arm to target pose (non-blocking)
    robot.arm.set_ee_pose_components(x=pose_target[0], y=pose_target[1], z=pose_target[2], roll=pose_target[3], pitch=pose_target[4], custom_guess=robot.arm.get_joint_commands(), blocking=False)