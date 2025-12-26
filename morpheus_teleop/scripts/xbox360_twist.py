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

class TeleopTwistJoy(TeleopTwist):
    def __init__(self):
        super().__init__()

        # Get twist topic
        self.twist_topic = rospy.get_param("~twist_topic", "twist_controller/command")

        # Initialize twist command publishers and joystick subscriber
        self.twist_pub = rospy.Publisher(self.twist_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.gripper_pub_robotiq = rospy.Publisher('robotiq_2f_85_gripper/control', Robotiq2FGripper_robot_output, queue_size=1)
        self.gripper_pub_onrobot = rospy.Publisher('onrobot_rg2ft/command', RG2FTCommand, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", sensor_msgs.msg.Joy, self.callback)

        # Initialize variables for holding joystick inputs
        self.joy_msg = None
        self.joy_msg_mutex = Lock()

        # Set scaling factor on inputs
        self.linear_scale = 0.05
        self.angular_scale = 0.05

        # Set limits on inputs and outputs
        self.input_min = 0.15
        self.input_max = 1.0
        self.output_max = 0.5

        # Initialize buffer arrays to enable moving average filtering of outputs
        self.twist_filter_size = 5  # Adjust this value for the moving average window size
        self.linear_x_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_y_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_z_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_x_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_y_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_z_buffer = deque(maxlen=self.twist_filter_size)
        self.buffer_list = [self.linear_x_buffer, self.linear_y_buffer, self.linear_z_buffer, self.angular_x_buffer, self.angular_y_buffer, self.angular_z_buffer]

        # Initialize moveit commander for giving motion plans to the robot, such as returning to home position
        self.robot_commander = moveit_commander.RobotCommander()
        self.planning_scene_interface = moveit_commander.PlanningSceneInterface()
        self.group_name = rospy.get_param("~arm_group", "arm")
        self.move_group_commander = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # Initialize joint commands for returning to home position
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.home = [0, -1.5708, 1.5708, -1.5708, -1.5708, 3.14]

    def get_joy_msg(self):
        # Retrieve joy_msg and return a safe copy
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()
        return msg

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

    def publish(self, rotated_axes, msg):
        # Publish pre-processed commands to the twist topic
        buttons = list(msg.buttons)

        # If menu button is pressed, move to home
        if buttons[7] == 1:
            self.moveto(joint_pos=self.home)
            return

        # Append inputs on each axis to the respective buffers
        buffer_zip = zip(self.buffer_list, rotated_axes)
        [buffer.append(axis) for buffer, axis in buffer_zip]

        # Obtain a moving average from each buffer and assign it to a new twist command
        twist = geometry_msgs.msg.Twist()
        twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z = [sum(buffer) / max(1,len(buffer)) for buffer in self.buffer_list]
        self.twist_pub.publish(twist)

        # Convert Xbox button press into open/close command
        # Inputs: 
        #   name            type                limits (Robotiq 2F-85)          limits(OnRobot RG2FT)
        #   position        float [0,50]        [0,50] mm                       [0,100] mm
        #   force           float [0,40]        [20,235] N, capped to 40        [0,40] N
        #   speed           float [0,150]       [20,150] mm/s                   (No speed control, ranges ~[180, 25] mm/s as position increases)
        position = None
        if buttons[0] == 1:
            position = 0 # Closed position, mm
        elif buttons[1] == 1:
            position = 50 # Open position, mm
        force = 20 # N
        speed = 100 # mm/s

        # Command Robotiq 2F-85 gripper
        if position is not None:
            command_robotiq2F85(publisher=self.gripper_pub_robotiq, position=position, force=force, speed=speed)

        # Command OnRobot RG2FT gripper
        if position is not None:
            command_onrobotRG2FT(publisher=self.gripper_pub_onrobot, position=position, force=force)

    def update(self, msg):
        if msg is not None:
            axes = self.msg_to_axes(msg)
            axes = self.rearrange_axes(axes)
            return axes
        else:
            return None

    def loop_once(self):
        if self.joy_msg is not None:
            axes = self.update(self.joy_msg)
            if axes is not None:
                self.publish(axes, self.joy_msg)

    def callback(self, msg):
        # Retrieve joy_msg
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()

    def moveto(self, joint_pos=None):
        # Command robot to move to specified joint position, such as home position
        if joint_pos is None:
            joint_pos = self.home
        # List expected twist and position controllers
        twist_controllers = ["twist_controller", "cartesian_compliance_controller"]
        pos_controllers = ["pos_joint_traj_controller"]

        # Retrieve list of active controllers
        controllers = list_controllers().controller
        running_controllers = [controller.name for controller in controllers if controller.state=="running"]
        running_twist_controllers = list(set(running_controllers) & set(twist_controllers))
        stopped_controllers = [controller.name for controller in controllers if (controller.state=="initialized" or controller.state=="stopped")]
        stopped_pos_controllers = list(set(stopped_controllers) & set(pos_controllers))

        try:
            # Switch to position controller
            print("Stopping controllers: " + str(running_twist_controllers))
            print("Starting controllers: " + str(stopped_pos_controllers))
            switch_controller(stop_controllers=running_twist_controllers, start_controllers=stopped_pos_controllers)
            print("Controller states: " + str([controller.state for controller in controllers]))

            # Execute a position command
            print("Executing move command")
            self.move_group_commander.go(joint_pos, wait=True)

            # Stop after completion
            print("Stopping after move command")
            self.move_group_commander.stop()

            # Reset target pose to match current pose


            # Switch back to twist controller
            print("Stopping controllers: " + str(stopped_pos_controllers))
            print("Starting controllers: " + str(running_twist_controllers))
            switch_controller(stop_controllers=stopped_pos_controllers, start_controllers=running_twist_controllers)
            print("Controller states: " + str([controller.state for controller in controllers]))
            return True
        except (...) as e:
            print("Twist controller moveto() failed: %s"%e)
            # If failed, attempt to restore status quo
            print("Attempting to restore normal state")
            print("Stopping move command")
            self.move_group_commander.stop()
            print("Stopping controllers: " + str(stopped_pos_controllers))
            print("Starting controllers: " + str(running_twist_controllers))
            switch_controller(stop_controllers=stopped_pos_controllers, start_controllers=running_twist_controllers, strictness=1)
            print("Controller states: " + str([controller.state for controller in controllers]))
            return False

if __name__ == '__main__':
    rospy.init_node('xbox360_twist')

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
