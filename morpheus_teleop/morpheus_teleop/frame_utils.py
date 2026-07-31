#! /usr/bin/env python3

# General Imports
import numpy as np
from scipy.spatial.transform import Rotation as R
# ROS Imports
from rclpy.time import Time
from rclpy.logging import get_logger
# TF2 ROS Imports
from tf2_ros import TransformException

# Rearrange the control axes to make the controls more intuitive
def rearrange_axes(command, logger=get_logger("frame_utils")):
    # Command is the 6 DOF motion control inputs for the robot
    rearranged_command = command
    try:
        r_control_linear = np.array([[ 1,  0,  0],
                                        [ 0,  1,  0],
                                        [ 0,  0,  1]])
        r_control_angular = np.array([[ 1,  0,  0],
                                        [ 0,  1,  0],
                                        [ 0,  0,  1]])

        rearranged_command = np.concatenate((np.matmul(r_control_linear, command[:3]), np.matmul(r_control_angular, command[3:])))
    except (ValueError) as e:
        logger.warn(f"Control axis rearrangement failed due to ValueError: {e}")
    return rearranged_command

# Perform coordinate transfer by rotating to target_frame from source_frame
def rotate_axes(tf_buffer, command, target_frame = "world", source_frame = "tcp_link"):
    # Command is the 6 DOF motion control inputs for the robot
    rotated_axes = np.array(command)

    # Rotate the axes of the end effector to be more intuitive
    effector_offset = R.from_matrix([[ 0,  1,  0],
                                        [ 0,  0,  1],
                                        [-1,  0,  0]])
    
    # Lookup the transform from source_frame to target_frame 
    transform = get_transform(tf_buffer, target_frame, source_frame)
    
    transform_quaternion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

    transform_rotation = R.from_quat(transform_quaternion)

    # Apply the rotation matrix
    offset_axes = np.concatenate((effector_offset.apply(command[:3]), effector_offset.apply(command[3:])))
    rotated_command = np.concatenate((transform_rotation.apply(offset_axes[:3]), transform_rotation.apply(offset_axes[3:])))
    rotated_command = np.multiply(rotated_axes, [-1, -1, 1, -1, -1, 1])
    return rotated_command

# Get the robot's current state
def get_joint_state(robot):
    return robot.get_joint_state()

# Calculate the yaw/waist rotation of the robot
def get_yaw(robot):
    robot_state = robot.get_joint_state()
    return robot_state[0]

# Get the robot's current end effector pose
def get_ee_pose(robot):
    return robot.move_group_commander.get_current_pose()

# Calculate the transform from source_frame to target_frame
def get_transform(tf_buffer, target_frame = "world", source_frame = "tcp_link", source_time=Time(), logger=get_logger("frame_utils")):
    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, source_time)
    except TransformException as e:
        logger.info(
            f'Could not transform from {source_frame} to {target_frame}: {e}')
        return
    return transform

# Calculate the transform of the end-effector w.r.t. the robot's shoulder/yaw position
def get_T_yb(self):
    T_ee = self.get_T(self.move_group_commander.get_end_effector_link())
    yaw = self.get_yaw()
    T_yaw = ang.yawToRotationMatrix(yaw)
    T_ee_yaw = np.dot(ang.transInv(T_yaw), T_ee)
    return T_ee_yaw