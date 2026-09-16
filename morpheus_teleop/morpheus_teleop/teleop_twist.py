#! /usr/bin/env python3

# General Imports
import numpy as np
# ROS Imports
from morpheus_teleop.utils import apply_twist, transform_to_pose, twist_to_wrench
from morpheus_teleop.utils import twist_to_wrench
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.time import Duration, Time
# TF2 ROS Imports
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# ROS Message Imports
from geometry_msgs.msg import TwistStamped, WrenchStamped, PoseStamped
from sensor_msgs.msg import Joy, JointState
from control_msgs.action import ParallelGripperCommand
# Individual Imports
from threading import Lock
from copy import deepcopy
from collections import deque
from scipy.spatial.transform import Rotation as R
# Local Imports
from joy_utils import joy_msg_to_dict, get_joy_msg
from frame_utils import rearrange_axes, rotate_axes, get_joint_state, get_yaw
from morpheus_msgs.action import MoveToNamedTarget

from teleop_base import TeleopBase

class TeleopTwist(TeleopBase):
    """ 
    Teleoperation class for interpreting joystick inputs as twist (translational and rotational velocity) commands.
    Receives inputs from controller drivers (ROS2 joy package).
    Converts buttons and axes to twist commands and other motion commands.
    Optionally transforms control frame between world frame and end effector frame.
    Sends outputs to robot drivers (UR, Interbotix) and gripper drivers (Robotiq, Onrobot).
    """
    def __init__(self, node_name="teleop_node"):
        super().__init__(node_name)

        # Set controller type
        self.controller_type = self.declare_parameter("controller_type", "ps4").value
        
        # Initialize frame swap variables
        self.frame_id = self.declare_parameter("frame_id", "base").value
        self.end_effector = self.declare_parameter("end_effector", "tool0").value
        self._already_swapped = False
        self._use_ee_frame = False
        
        # Initialize trajectory control
        self._move_to_named_target_action_client = ActionClient(self, MoveToNamedTarget, 'move_to_named_target')
        self._move_to_named_target_response_future = Future()
        self._move_to_named_target_result_future = Future()
        self._waiting = False

        # Listen for robot state
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_stamped = None

        # Declare command messages
        self.twist_stamped = TwistStamped()
        self.wrench_stamped = WrenchStamped()
        self.pose_stamped = PoseStamped()
        self.gripper_command = JointState()

        # Get topics
        self.joy_topic = self.declare_parameter("joy_topic", "/joy").value
        self.twist_topic = self.declare_parameter("twist_topic", "target_twist").value
        self.wrench_topic = self.declare_parameter("wrench_topic", "target_wrench").value
        self.pose_topic = self.declare_parameter("pose_topic", "target_frame").value
        self.gripper_topic = self.declare_parameter("gripper_topic", "gripper_command").value

        # Initialize joystick subscriber and twist command publishers 
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 1)
        self.wrench_pub = self.create_publisher(WrenchStamped, self.wrench_topic, 1)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 1)
        self.gripper_pub = self.create_publisher(JointState, self.gripper_topic, 1)

        # Initialize variables for holding joystick inputs
        self.joy_msg = None
        self.joy_msg_mutex = Lock()
        self.input_dict = {}

        # Set scaling factor on inputs
        self.linear_scale = 2
        self.angular_scale = 10

        # Set limits on raw inputs and outputs
        self.input_min = 0.05 # Deadzone for joystick inputs
        self.input_max = 1.0
        self.output_max = 10.0

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
        # If menu button is pressed, move to home
        if input_dict["HOME_POSE"] == 1:
            self.get_logger().info("Attempting trajectory to home position")
            self.move_to_named_target("home")
            return
        if input_dict["SLEEP_POSE"] == 1:
            self.get_logger().info("Attempting trajectory to up position")
            self.move_to_named_target("up")
            return

        self.twist_pub.publish(self.twist_stamped)
        self.wrench_pub.publish(self.wrench_stamped)
        self.pose_pub.publish(self.pose_stamped)
        self.gripper_pub.publish(self.gripper_command)
    
    def get_transform(self, target_frame=None, source_frame=None, time=Time(), timeout=Duration(seconds=1)):
        """ Convenience function to lookup a transform from the tf_buffer with error handling. """
        target_frame = target_frame if target_frame is not None else self.frame_id
        source_frame = source_frame if source_frame is not None else self.end_effector
        try:
            tf_stamped = self.tf_buffer.lookup_transform(target_frame=target_frame, source_frame=source_frame, time=time, timeout=timeout)
            return tf_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"Exception in lookup_transform for twist_to_pose: {e}")
        except Exception as e:
            self.get_logger().warn(f"Failed lookup_transform for twist_to_pose: {e}")
    
    def get_mean_command(self):
        # Obtain a moving average from each buffer
        return [np.mean(np.array(buffer)) for buffer in self.buffer_list]
            
    def update_buffer(self, input_dict):
        # Get command based on the input dictionary
        command = [input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_PITCH"], input_dict["EE_ROLL"], input_dict["WAIST"]]
        # Append inputs on each axis to the respective buffers
        for i, buffer in enumerate(self.buffer_list):
            buffer.append(command[i])
    
    def update_twist(self, command, stamp=Time().to_msg()):
        # Assign a new twist command
        twist = TwistStamped()
        twist.header.stamp                      = stamp
        twist.header.frame_id                   = self.frame_id
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = command
        self.twist_stamped = twist
        return self.twist_stamped

    def update_wrench(self, twist_stamped):
        # Update wrench based on twist
        wrench = WrenchStamped()
        wrench.header.stamp        = twist_stamped.header.stamp
        wrench.header.frame_id     = self.frame_id
        wrench.wrench              = twist_to_wrench(twist_stamped.twist, scaling_factor=1)
        self.wrench_stamped = wrench
        return self.wrench_stamped

    def update_pose(self, twist_stamped, tf_stamped, dt):
        # Update pose based on linear velocity and dt
        pose = PoseStamped()
        pose.header.stamp          = twist_stamped.header.stamp
        pose.header.frame_id       = self.frame_id
        pose.pose                  = apply_twist(twist_stamped.twist, transform_to_pose(tf_stamped.transform), dt=dt)
        self.pose_stamped = pose
        return self.pose_stamped

    def update_gripper(self, input_dict):
        # Update gripper command based on button inputs
        gripper_command = JointState()
        gripper_command.name = ["gripper_joint"]
        gripper_command.position = [(1 + input_dict["GRIPPER_CLOSE"] - input_dict["GRIPPER_OPEN"]) / 2] # 1 = closed, 0 = open
        gripper_command.velocity = [0.05] # m/s
        gripper_command.effort = [20] # N
        self.gripper_command = gripper_command
    
    def update_input_dict(self, joy_msg):
        input_dict = joy_msg_to_dict(
            joy_msg, 
            linear_scale=self.linear_scale, 
            angular_scale=self.angular_scale, 
            input_min=self.input_min, 
            input_max=self.input_max, 
            output_max=self.output_max,
            logger=self.get_logger())
        input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"] = \
            rearrange_axes([input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"]])
        self.input_dict = input_dict
        return input_dict

    def update(self, joy_msg):
        try:
            # If waiting on an action call, do nothing
            if self._waiting:
                return
            
            # Get the latest transform from the end effector to the base frame
            tf_stamped = self.get_transform(
                target_frame = self.frame_id, 
                source_frame = self.end_effector, 
                time=Time())

            # Update commands
            # A constant scaling factor is used for update_pose instead of dt,
            # since the target pose is calculated each step from the tf transform 
            # and not based on the previous target pose
            self.update_input_dict(joy_msg)
            self.update_buffer(self.input_dict)
            self.update_twist(self.get_mean_command(), stamp=joy_msg.header.stamp)
            self.update_wrench(self.twist_stamped)
            self.update_pose(self.twist_stamped, tf_stamped, 0.01)
            self.update_gripper(self.input_dict)
            self.publish(self.input_dict)
        except Exception as e:
            self.get_logger().warn(f"Exception in update for teleop_twist: {e}")

    def joy_callback(self, msg):
        # Retrieve joy_msg and store a safe copy
        self.joy_msg_mutex.acquire()
        self.joy_msg = deepcopy(msg)
        self.joy_msg_mutex.release()
        # Process and publish
        self.update(self.joy_msg)
    
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
