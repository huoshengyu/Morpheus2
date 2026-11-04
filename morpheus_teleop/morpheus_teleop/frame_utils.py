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
from scipy.spatial.transform import Rotation as R
# ROS Imports
from rclpy.logging import get_logger
# TF2 ROS Imports

# Rearrange the control axes with a fixed rotation to make the controls more intuitive
def rearrange_command(command, logger=get_logger("frame_utils")):
    # Command is the 6 DOF motion control inputs for the robot
    rearranged_command = command
    try:
        r_control_linear =  R.from_matrix([[ 1,  0,  0],
                                           [ 0,  0, -1],
                                           [ 0,  1,  0]])
        r_control_angular = R.from_matrix([[ 0,  1,  0],
                                           [-1,  0,  0],
                                           [ 0,  0,  1]])

        rearranged_command = np.concatenate((r_control_linear.apply(command[:3]), r_control_angular.apply(command[3:])))
        rearranged_command = np.multiply(rearranged_command, [1, 1, 1,-1, 1, 1])
    except (ValueError) as e:
        logger.warn(f"Control axis rearrangement failed due to ValueError: {e}")
    return rearranged_command

# Perform coordinate transfer by dynamically rotating to target_frame from source_frame
def transform_command(command, tf_stamped, logger=get_logger("frame_utils")):
    try:
        # Lookup the transform from source_frame to target_frame 
        transform_quaternion = [tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w]
        transform_rotation = R.from_quat(transform_quaternion)

        # Apply the rotation matrix
        rearranged_command = np.multiply(command, [ 1,-1,-1, 1,-1,-1])
        transformed_command = np.concatenate((transform_rotation.apply(rearranged_command[:3]), transform_rotation.apply(rearranged_command[3:])))
    except (ValueError) as e:
        logger.warn(f"Control frame transformation failed due to ValueError: {e}")
    return transformed_command