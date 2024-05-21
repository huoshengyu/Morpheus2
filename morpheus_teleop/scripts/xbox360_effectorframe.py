#! /usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
from threading import Lock
from copy import deepcopy
from collections import deque
from scipy.spatial.transform import Rotation as R
from scipy.linalg import inv
from scipy import pi

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

from xbox360_twist import TeleopTwistJoy

class TeleopTwistJoyEffectorFrame(TeleopTwistJoy):
    def rotate_axes(self, rearranged_axes, target_frame = "world", source_frame = "tcp_link"):
        # Perform coordinate transfer by rotating to target_frame from source_frame
        rotated_axes = np.array(rearranged_axes)

        effector_offset = R.from_matrix([[ 0,  1,  0],
                                         [ 0,  0,  1],
                                         [-1,  0,  0]])
        
        # Lookup the transform (target_frame) from (source_frame)
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        
        effector_quaternion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

        effector_rotation = R.from_quat(effector_quaternion)

        # Apply the rotation matrix
        offset_axes = np.concatenate((effector_offset.apply(rearranged_axes[:3]), effector_offset.apply(rearranged_axes[3:])))
        rotated_axes = np.concatenate((effector_rotation.apply(offset_axes[:3]), effector_rotation.apply(offset_axes[3:])))
        rotated_axes = np.multiply(rotated_axes, [-1, -1, 1, -1, -1, 1])
        return rotated_axes

    def rearrange_axes(self, scaled_axes):
        rearranged_axes = scaled_axes
        # Rearrange the control axes to make the controls more intuitive
        try:
            # Rearrange the axes to make the controls more intuitive
            r_control_linear = np.array([[ 1,  0,  0],
                                            [ 0,  1,  0],
                                            [ 0,  0,  1]])
            r_control_angular = np.array([[ 1,  0,  0],
                                            [ 0,  1,  0],
                                            [ 0,  0,  1]])

            rearranged_axes = np.concatenate((np.matmul(r_control_linear, scaled_axes[:3]), np.matmul(r_control_angular, scaled_axes[3:])))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Control axis rearrangement failed!")
        return rearranged_axes

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('xbox360_twist')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    teleop_twist_joy = TeleopTwistJoyEffectorFrame()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
