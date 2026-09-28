#! /usr/bin/env python3

# General Imports
import numpy as np
# ROS Imports
from morpheus_teleop.utils import apply_twist, transform_to_pose
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
# Individual Imports
from threading import Lock
from copy import deepcopy
from collections import deque
# Local Imports
from joy_utils import joy_msg_to_dict
from frame_utils import rearrange_command, transform_command
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
        self.tf_stamped = tf2_ros.TransformStamped()

        # Declare command messages
        self.command = np.zeros(6)  # [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
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
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self._joy_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 1)
        self.wrench_pub = self.create_publisher(WrenchStamped, self.wrench_topic, 1)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 1)
        self.gripper_pub = self.create_publisher(JointState, self.gripper_topic, 1)

        # Initialize variables for holding joystick inputs
        self.joy_msg = Joy()
        self.joy_msg_mutex = Lock()
        self.input_dict = {}

        # Set scaling factor on inputs
        self.linear_scale = 2
        self.angular_scale = 10
        # A constant scaling factor is used for update_pose instead of dynamic dt,
        # since the target pose is calculated each step from the tf transform 
        # and not based on the previous target pose
        self.dt = 0.05

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

    def update(self, joy_msg: Joy) -> None:
        try:
            # If waiting on an action to complete, do nothing
            if self._waiting:
                return
            # Get the latest transform from the end effector to the base frame
            self.tf_stamped = self.get_transform(
                target_frame = self.frame_id, 
                source_frame = self.end_effector, 
                time=Time())
            # Update commands
            self.update_input_dict(joy_msg)
            self.update_frame()
            self.update_buffer_list()
            self.update_command()
            self.update_twist()
            self.update_wrench()
            # A constant scaling factor is used for update_pose instead of dt,
            # since the target pose is calculated each step from the tf transform 
            # and not based on the previous target pose
            self.update_pose()
            self.update_gripper()
            self.publish()
        except Exception as e:
            self.get_logger().warn(f"Exception in update for teleop_twist: {e}")
    
    def update_input_dict(self, joy_msg: Joy) -> dict:
        # Convert joy_msg to input_dict
        input_dict = joy_msg_to_dict(
            joy_msg, 
            linear_scale=self.linear_scale, 
            angular_scale=self.angular_scale, 
            input_min=self.input_min, 
            input_max=self.input_max, 
            output_max=self.output_max,
            logger=self.get_logger())
        input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"] = \
            rearrange_command([input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"]])
        self.input_dict = input_dict
        return input_dict
    
    def update_frame(self) -> None:
        # Update the control frame based on the current input dictionary
        if self.input_dict["FRAME_SWAP"] == 1 and not self._already_swapped:
            self._use_ee_frame = not self._use_ee_frame
            self._already_swapped = True
            self.get_logger().info("Swapping control frame to {frame} frame.".format(frame=self.end_effector if self._use_ee_frame else self.frame_id))
        elif self.input_dict["FRAME_SWAP"] == 0:
            self._already_swapped = False
            
    def update_buffer_list(self) -> None:
        # Get command based on the input dictionary
        command = [self.input_dict["EE_X"], self.input_dict["EE_Y"], self.input_dict["EE_Z"], self.input_dict["EE_ROLL"], self.input_dict["EE_PITCH"], self.input_dict["WAIST"]]
        if self._use_ee_frame:
            # Transform command from end effector frame to base frame
            command = transform_command(command, self.tf_stamped, logger=self.get_logger())
        # Append inputs on each axis to the respective buffers
        for i, buffer in enumerate(self.buffer_list):
            buffer.append(command[i])
    
    def update_command(self) -> None:
        # Update the command based on the current buffer values
        self.command = self.get_mean_command()

    def update_twist(self) -> TwistStamped:
        # Assign a new twist command
        twist = TwistStamped()
        twist.header.stamp                      = self.tf_stamped.header.stamp
        twist.header.frame_id                   = self.frame_id
        twist.twist.linear.x                    = self.command[0]
        twist.twist.linear.y                    = self.command[1]
        twist.twist.linear.z                    = self.command[2]
        twist.twist.angular.x                   = self.command[3]
        twist.twist.angular.y                   = self.command[4]
        twist.twist.angular.z                   = self.command[5]
        self.twist_stamped = twist
        return self.twist_stamped

    def update_wrench(self) -> WrenchStamped:
        # Update wrench based on twist
        wrench = WrenchStamped()
        wrench.header.stamp                     = self.tf_stamped.header.stamp
        wrench.header.frame_id                  = self.frame_id
        wrench.wrench.force.x                   = self.command[0] * self.dt
        wrench.wrench.force.y                   = self.command[1]
        wrench.wrench.force.z                   = self.command[2]
        wrench.wrench.torque.x                  = self.command[3]
        wrench.wrench.torque.y                  = self.command[4]
        wrench.wrench.torque.z                  = self.command[5]
        self.wrench_stamped = wrench
        return self.wrench_stamped

    def update_pose(self) -> PoseStamped:
        # Update pose based on linear velocity and dt
        pose = PoseStamped()
        pose.header.stamp          = self.tf_stamped.header.stamp
        pose.header.frame_id       = self.frame_id
        pose.pose                  = apply_twist(self.twist_stamped.twist, transform_to_pose(self.tf_stamped.transform), dt=self.dt)
        self.pose_stamped = pose
        return self.pose_stamped

    def update_gripper(self) -> None:
        # Update gripper command based on button inputs
        gripper_command = JointState()
        gripper_command.name = ["gripper_joint"]
        gripper_command.position = [(1 + self.input_dict["GRIPPER_CLOSE"] - self.input_dict["GRIPPER_OPEN"]) / 2] # 1 = closed, 0 = open
        gripper_command.velocity = [0.05] # m/s
        gripper_command.effort = [5] # N
        self.gripper_command = gripper_command

    def publish(self) -> None:
        # If menu button is pressed, move to home
        if self.input_dict["HOME_POSE"] == 1:
            self.move_to_named_target("home")
        elif self.input_dict["SLEEP_POSE"] == 1:
            self.move_to_named_target("up")

        self.twist_pub.publish(self.twist_stamped)
        self.wrench_pub.publish(self.wrench_stamped)
        self.pose_pub.publish(self.pose_stamped)
        self.gripper_pub.publish(self.gripper_command)
    
    def get_transform(self, target_frame=None, source_frame=None, time=Time(), timeout=Duration(seconds=1)) -> tf2_ros.TransformStamped:
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
    
    def get_mean_command(self) -> np.ndarray:
        # Obtain a moving average from each buffer
        return np.array([np.mean(np.array(buffer)) for buffer in self.buffer_list])
    
    def move_to_named_target(self, name) -> bool:
        # Check if the action server is ready
        if self._move_to_named_target_action_client is None or not self._move_to_named_target_action_client.server_is_ready():
            self.get_logger().info("Move to named target action server is not ready. Skipping move to named target command.", throttle_duration_sec=1.0)
            return False
        self.get_logger().info("Attempting trajectory to position: " + name)
        
        # Create goal message to move to named position
        goal_msg = MoveToNamedTarget.Goal()
        goal_msg.target_name = name
        
        # Send goal to action server
        self._move_to_named_target_action_client.wait_for_server()
        self._move_to_named_target_response_future = self._move_to_named_target_action_client.send_goal_async(goal_msg)
        self._move_to_named_target_response_future.add_done_callback(self._move_to_named_target_response_callback)
        return True
    
    def _move_to_named_target_response_callback(self, future) -> None:
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
        self._move_to_named_target_result_future.add_done_callback(self._move_to_named_target_result_callback)
    
    def _move_to_named_target_result_callback(self, future) -> None:
        # Stop waiting and allow controls to resume publishing, regardless of result
        result = future.result().result
        self.get_logger().info('Move to named target result: {0}'.format(result.error_code))
        self._waiting = False

    def _joy_callback(self, msg) -> None:
        # Retrieve joy_msg and store a safe copy
        self.joy_msg_mutex.acquire()
        self.joy_msg = deepcopy(msg)
        self.joy_msg_mutex.release()
        # Process and publish
        self.update(self.joy_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        
        teleop_twist = TeleopTwist()
        rclpy.spin_once(teleop_twist) # Ensures tf_buffer.lookup_transform() works before control loop begins
        rclpy.spin(teleop_twist)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
