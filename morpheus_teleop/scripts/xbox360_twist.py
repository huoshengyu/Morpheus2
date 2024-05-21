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

class TeleopTwistJoy():
    def __init__(self):
        twist_topic = rospy.get_param("~twist_topic", "/joy/twist")

        self.twist_pub = node.create_publisher(geometry_msgs.msg.Twist, queue_size=1, twist_topic)
        self.gripper_pub = node.create_publisher(Robotiq2FGripper_robot_output, queue_size=1, '/robotiq_2f_85_gripper/control')
        self.joy_sub = node.create_subscription(sensor_msgs.msg.Joy, "/joy", self.callback)
        
        self.joy_msg = None
        self.joy_msg_mutex = Lock()

        self.linear_scale = 0.05
        self.angular_scale = 0.05

        self.twist_filter_size = 5  # Adjust this value for the moving average window size
        self.linear_x_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_y_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_z_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_x_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_y_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_z_buffer = deque(maxlen=self.twist_filter_size)

    def get_joy_msg(self):
        # Retrieve joy_msg and return a safe copy
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()
        return msg

    def msg_to_axes(self, msg, linear_scale=0, angular_scale=0):
        # Convert joy_msg inputs to command outputs in 6 DOF
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        for i in range(len(axes)):
            if abs(axes[i]) < 0.15:
                axes[i] = 0
            if abs(axes[i]) > 1:
                axes[i] = axes[i] / abs(axes[i])

        if linear_scale == 0:
            linear_scale = self.linear_scale
        if angular_scale == 0:
            angular_scale = self.angular_scale

        # Left stick: x = -axes[0], y = axes[1]
        # Right stick: x = -axes[3], y = axes[4]
        # Triggers: LT = axes[2], RT = axes[5]
        # Dpad: L/R = -axes[6], U/D = axes[7]
        # Buttons: [A, B, X, Y, LB, RB, Share, Menu, Xbox, Lstick, Rstick]
        # scaled_axes = [-axes[1] * linear_scale, 
        #                 -axes[0] * linear_scale, 
        #                 ((1 - (axes[5] + 1) / 2) - (1 - (axes[2] + 1) / 2)) * linear_scale, 
        #                 -axes[3] * angular_scale, 
        #                 axes[4] * angular_scale, 
        #                 -(buttons[5] - buttons[4]) * angular_scale]

        # Button remapping so that left side = translation, right side = rotation
        
        scaled_axes = [ axes[0] * linear_scale, 
                        -axes[1] * linear_scale, 
                        ((buttons[4]) - (axes[2] < 0.0)) * linear_scale, 
                        -axes[4] * angular_scale, 
                        -axes[3] * angular_scale, 
                        -((buttons[5]) - (axes[5] < 0.0)) * angular_scale]

        # Enforce a safety limit on speed
        for i in range(len(scaled_axes)):
            if abs(scaled_axes[i]) > 0.5:
                scaled_axes[i] = 0.5 * scaled_axes[i] / abs(scaled_axes[i])
        
        return scaled_axes

    def rearrange_axes(self, scaled_axes):
        rearranged_axes = scaled_axes
        # Rearrange the control axes to make the controls more intuitive
        try:
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
    
    def rotate_axes(self, rearranged_axes, target_frame = "world", source_frame = "tcp_link"):
        # Perform coordinate transfer by rotating from 
        rotated_axes = rearranged_axes
        return rotated_axes

    def publish(self, rotated_axes, msg):
        # Publish pre-processed commands to the twist topic
        buttons = list(msg.buttons)

        # Old command code, kept here for reference only
        # self.linear_x_buffer.append(axes[1] * linear_scale)
        # self.linear_y_buffer.append(axes[0] * linear_scale)
        # self.linear_z_buffer.append(((1 - (axes[5] + 1) / 2) - (1 - (axes[2] + 1) / 2)) * linear_scale)
        # self.angular_x_buffer.append(-axes[3] * angular_scale)
        # self.angular_y_buffer.append(axes[4] * angular_scale)
        # self.angular_z_buffer.append((buttons[4] - buttons[5]) * angular_scale)

        self.linear_x_buffer.append(rotated_axes[0])
        self.linear_y_buffer.append(rotated_axes[1])
        self.linear_z_buffer.append(rotated_axes[2])
        self.angular_x_buffer.append(rotated_axes[3])
        self.angular_y_buffer.append(rotated_axes[4])
        self.angular_z_buffer.append(rotated_axes[5])

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = sum(self.linear_x_buffer) / len(self.linear_x_buffer)
        twist.linear.y = sum(self.linear_y_buffer) / len(self.linear_y_buffer)
        twist.linear.z = sum(self.linear_z_buffer) / len(self.linear_z_buffer)
        twist.angular.x = sum(self.angular_x_buffer) / len(self.angular_x_buffer)
        twist.angular.y = sum(self.angular_y_buffer) / len(self.angular_y_buffer)
        twist.angular.z = sum(self.angular_z_buffer) / len(self.angular_z_buffer)

        command = Robotiq2FGripper_robot_output()
        command.rACT = 0x1
        command.rGTO = 0x1 # go to position
        command.rATR = 0x0 # No emergency release
        command.rSP = 128 # speed
        command.rPR = 0x0 # position
        command.rFR = 30 # effort
        if buttons[0] == 1:
            command.rPR = 230 # position
            self.gripper_pub.publish(command)
        elif buttons[1] == 1:
            command.rPR = 0 # position
            self.gripper_pub.publish(command)

        self.twist_pub.publish(twist)

    def update(self, msg):
        if msg is not None:
            scaled_axes = self.msg_to_axes(msg)
            rearranged_axes = self.rearrange_axes(scaled_axes)
            rotated_axes = self.rotate_axes(rearranged_axes)
            return rotated_axes
        else:
            return None

    def loop_once(self):
        if self.joy_msg is not None:
            rotated_axes = self.update(self.joy_msg)
            if rotated_axes is not None:
                self.publish(rotated_axes, self.joy_msg)

    def callback(self, msg):
        # Retrieve joy_msg
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('xbox360_twist')

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
