#! /usr/bin/env python3

# General Packages
import numpy as np
# ROS Packages
import rospy
import tf2_ros
import moveit_commander
# ROS Messages
import geometry_msgs.msg   
import sensor_msgs.msg 
import moveit_msgs.msg
# Individual Imports
from threading import Lock
from copy import deepcopy
from collections import deque
# Local Imports
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand
from utils import switch_controller, list_controllers, command_robotiq2F85, command_onrobotRG2FT

from teleop_twist import TeleopTwist

### Button mappings from xsarm_joy.cpp
# PS3 Controller button mappings
ps3 = {{"GRIPPER_PWM_DEC", 0}, # buttons start here
        {"GRIPPER_OPEN", 1},
        {"GRIPPER_PWM_INC", 2},
        {"GRIPPER_CLOSE", 3},
        {"EE_Y_INC", 4},
        {"EE_Y_DEC", 5},
        {"WAIST_CCW", 6},
        {"WAIST_CW", 7},
        {"SLEEP_POSE", 8},
        {"HOME_POSE", 9},
        {"TORQUE_ENABLE", 10},
        {"FLIP_EE_X", 11},
        {"FLIP_EE_ROLL", 12},
        {"SPEED_INC", 13},
        {"SPEED_DEC", 14},
        {"SPEED_COARSE", 15},
        {"SPEED_FINE", 16},
        {"EE_X", 0},            # axes start here
        {"EE_Z", 1},
        {"EE_ROLL", 3},
        {"EE_PITCH", 4}}

# PS4 Controller button mappings
ps4 = {{"GRIPPER_PWM_DEC", 0}, # buttons start here
        {"GRIPPER_OPEN", 1},
        {"GRIPPER_PWM_INC", 2},
        {"GRIPPER_CLOSE", 3},
        {"EE_Y_INC", 4},
        {"EE_Y_DEC", 5},
        {"WAIST_CCW", 6},
        {"WAIST_CW", 7},
        {"SLEEP_POSE", 8},
        {"HOME_POSE", 9},
        {"TORQUE_ENABLE", 10},
        {"FLIP_EE_X", 11},
        {"FLIP_EE_ROLL", 12},
        {"EE_X", 0},            # axes start here
        {"EE_Z", 1},
        {"EE_ROLL", 3},
        {"EE_PITCH", 4},
        {"SPEED_TYPE", 6},
        {"SPEED", 7}}

# Xbox 360 Controller button mappings
xbox360 = {{"GRIPPER_PWM_DEC", 0}, # buttons start here
        {"GRIPPER_OPEN", 1},
        {"GRIPPER_CLOSE", 2},
        {"GRIPPER_PWM_INC", 3},
        {"WAIST_CCW", 4},
        {"WAIST_CW", 5},
        {"SLEEP_POSE", 6},
        {"HOME_POSE", 7},
        {"TORQUE_ENABLE", 8},
        {"FLIP_EE_X", 9},
        {"FLIP_EE_ROLL", 10},
        {"EE_X", 0},            # axes start here
        {"EE_Z", 1},
        {"EE_Y_INC", 2},
        {"EE_ROLL", 3},
        {"EE_PITCH", 4},
        {"EE_Y_DEC", 5},
        {"SPEED_TYPE", 6},
        {"SPEED", 7}}

class TeleopTwistJoy(TeleopTwist):
    def __init__(self):
        super().__init__()

    def msg_to_axes(self, msg, linear_scale=None, angular_scale=None):
        # Convert joy_msg inputs to command outputs in 6 DOF
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        # Set a deadzone and a limit in input values
        for i in range(len(axes)):
            if abs(axes[i]) < self.input_min:
                axes[i] = 0
            if abs(axes[i]) > self.input_max:
                axes[i] = axes[i] / abs(axes[i])

        if linear_scale == None:
            linear_scale = self.linear_scale
        if angular_scale == None:
            angular_scale = self.angular_scale

        # Button remapping so that left joystick/buttons = translation, right joystick/buttons = rotation
        scaled_axes = np.array([ axes[0] * linear_scale, 
                                -axes[1] * linear_scale, 
                                ((buttons[4]) - (axes[2] < 0.0)) * linear_scale, 
                                -axes[4] * angular_scale, 
                                -axes[3] * angular_scale, 
                                -((buttons[5]) - (axes[5] < 0.0)) * angular_scale])

        # Enforce a safety limit on speed
        scaled_axes = np.clip(scaled_axes, -self.output_max, self.output_max)
        
        return scaled_axes

if __name__ == '__main__':
    rospy.init_node('ps4_twist')

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
