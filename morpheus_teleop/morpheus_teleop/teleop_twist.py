#! /usr/bin/env python3

### Twist class for teleoperation ###
# Receives inputs from controller drivers (ROS2 joy package).
# Converts buttons and axes to twist commands and other motion commands.
# Optionally transforms control frame between world frame and end effector frame.
# Sends outputs to robot drivers (UR, Interbotix) and gripper drivers (Robotiq, Onrobot).

# General Imports
import sys
import numpy as np
# ROS Imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
# TF2 ROS Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# ROS Message Imports
import geometry_msgs.msg   
import sensor_msgs.msg 
import moveit_msgs.msg
from control_msgs.msg import GripperCommand
# Individual Imports
from threading import Lock
from copy import deepcopy
from collections import deque
from scipy.spatial.transform import Rotation as R
# Local Imports
from joy_utils import joy_msg_to_dict, get_joy_msg
from frame_utils import rearrange_axes, rotate_axes, get_joint_state, get_yaw
from trajectory_component import TrajectoryComponent
from morpheus_msgs.action import MoveToNamedTarget

# from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
# from onrobot_rg2ft_msgs.msg import RG2FTCommand

from teleop_base import TeleopBase

class TeleopTwist(TeleopBase):
    def __init__(self, node_name="teleop_node"):
        super().__init__(node_name)

        # Set controller type
        self.controller_type = self.declare_parameter("controller_type", "ps4").value
        
        # Initialize frame swap variables
        self.reference_frame = "base"
        self.already_swapped = False
        
        # Initialize trajectory control
        self._move_to_named_target_action_client = ActionClient(self, MoveToNamedTarget, 'move_to_named_target')
        self._move_to_named_target_response_future = Future()
        self._move_to_named_target_result_future = Future()
        self._waiting = False

        # Set robot model
        self.robot_model = self.declare_parameter("robot_model", "ur5e").value

        # Set gripper type
        self.gripper_type = self.declare_parameter("gripper_type", "robotiq").value

        # Listen for robot state
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Get twist topic
        self.twist_topic = self.declare_parameter("twist_topic", "twist_controller/command").value
        self.wrench_topic = self.declare_parameter("wrench_topic", "target_wrench").value

        # Initialize twist command publishers and joystick subscriber
        self.twist_pub = self.create_publisher(geometry_msgs.msg.Twist, self.twist_topic, 1)
        self.wrench_pub = self.create_publisher(geometry_msgs.msg.WrenchStamped, self.wrench_topic, 1)
        # self.gripper_pub_robotiq = self.create_publisher(Robotiq2FGripper_robot_output, 'robotiq_2f_85_gripper/control', 1)
        # self.gripper_pub_onrobot = self.create_publisher(RG2FTCommand, 'onrobot_rg2ft/command', 1)
        # self.gripper_pub_gazebo = ActionClient(self, GripperCommand, 'gripper_action_controller/gripper_cmd')
        self.joy_sub = self.create_subscription(sensor_msgs.msg.Joy, "joy", self.joy_callback, 10)

        # Initialize variables for holding joystick inputs
        self.joy_msg = None
        self.joy_msg_mutex = Lock()
        self.input_dict = {}

        # Set limits on raw inputs and outputs
        self.input_min = 0.05
        self.input_max = 1.0
        self.output_max = 1.0

        # Set scaling factor on inputs
        self.linear_scale = 1
        self.angular_scale = 1

        # Initialize buffer arrays to enable moving average filtering of outputs
        self.twist_filter_size = 5  # Adjust this value for the moving average window size
        self.linear_x_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_y_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_z_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_x_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_y_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_z_buffer = deque(maxlen=self.twist_filter_size)
        self.buffer_list = [self.linear_x_buffer, self.linear_y_buffer, self.linear_z_buffer, self.angular_x_buffer, self.angular_y_buffer, self.angular_z_buffer]

    def publish(self, input_dict):
        # Publish pre-processed commands to the twist topic
        command = [input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"]]

        # If waiting on an action call, do nothing
        if self._waiting:
            return

        # If menu button is pressed, move to home
        if input_dict["HOME_POSE"] == 1:
            self.get_logger().info("Attempting trajectory to home position")
            self.move_to_named_target("home")
            return
        if input_dict["SLEEP_POSE"] == 1:
            self.get_logger().info("Attempting trajectory to up position")
            self.move_to_named_target("up")
            return

        # Append inputs on each axis to the respective buffers
        buffer_zip = zip(self.buffer_list, command)
        [buffer.append(axis) for buffer, axis in buffer_zip]

        # Obtain a moving average from each buffer and assign it to a new twist command
        mean_command = [np.mean(np.array(buffer)) for buffer in self.buffer_list]
        twist = geometry_msgs.msg.Twist()
        twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z = mean_command
        self.twist_pub.publish(twist)
        
        # Obtain a moving average from each buffer and assign it to a new wrench command
        wrench = geometry_msgs.msg.WrenchStamped()
        wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z, wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z = mean_command
        self.wrench_pub.publish(wrench)

    def update(self, msg):
        if msg is not None:
            input_dict = joy_msg_to_dict(msg, controller_type=self.controller_type, logger=self.get_logger())
            input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"] = \
                rearrange_axes([input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"]])
            self.input_dict = input_dict
            return input_dict
        else:
            return None

    def loop_once(self):
        if self.joy_msg:
            self.update(self.joy_msg)
            if self.input_dict:
                self.publish(self.input_dict)

    def joy_callback(self, msg):
        # Retrieve joy_msg and store a safe copy
        self.joy_msg_mutex.acquire()
        self.joy_msg = deepcopy(msg)
        self.joy_msg_mutex.release()
        # Process and publish
        self.loop_once()
    
    def move_to_named_target(self, name):
        # Create goal message to move to named position
        goal_msg = MoveToNamedTarget.Goal()
        goal_msg.target_name = name
        
        # Send goal to action server
        self._move_to_named_target_action_client.wait_for_server()
        self._move_to_named_target_response_future = self._move_to_named_target_action_client.send_goal_async(goal_msg)
        self._move_to_named_target_response_future.add_done_callback(self.move_to_named_target_response_callback)
    
    def move_to_named_target_response_callback(self, future):
        # Set wait variable to True if the action goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Move to named target rejected')
            return
        self.get_logger().info('Move to named target accepted')
        self._waiting = True
            
        # Empty the buffers to prevent a sudden jump in commands when teleop is resumed
        [[buffer.append(0) for buffer in self.buffer_list] for _ in range(self.twist_filter_size)]

        # Get the action result future so the result callback can process the result
        self._move_to_named_target_result_future = goal_handle.get_result_async()
        self._move_to_named_target_result_future.add_done_callback(self.move_to_named_target_result_callback)
    
    def move_to_named_target_result_callback(self, future):
        # Stop waiting and allow controls to resume publishing, regardless of result
        result = future.result().result
        self.get_logger().info('Move to named target result: {0}'.format(result.error_code))
        self._waiting = False


def main(args=None):
    try:
        rclpy.init(args=args)
        
        teleop_twist = TeleopTwist()

        rclpy.spin(teleop_twist)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
