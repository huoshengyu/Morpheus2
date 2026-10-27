#!/usr/bin/env python3
"""
FACTR Teleop subclass for UR5e using ROS 2 topics.

- Subscribes to follower joint states (default: /joint_states)
- Publishes follower joint position commands as sensor_msgs/JointState to a configurable topic
- Reads follower joint efforts (if available) for torque feedback
- Optional gripper feedback topic
"""

import time
import numpy as np

import rclpy
from sensor_msgs.msg import JointState

from factr_teleop.factr_teleop import FACTRTeleop


class FACTRTeleopUR5eROS2(FACTRTeleop):
    def __init__(self):
        super().__init__()
        # optional params from YAML under a `communication` section (defaults provided)
        comm_cfg = self.config.get("communication", {})
        self.follower_joint_state_topic = comm_cfg.get("follower_joint_state_topic", "/joint_states")
        self.follower_command_topic = comm_cfg.get("follower_command_topic", "/ur5e/joint_commands")
        self.follower_gripper_state_topic = comm_cfg.get("follower_gripper_state_topic", None)
        # list of follower joint names expected by the follower controller (optional)
        self.follower_joint_names = self.config.get("arm_teleop", {}).get("follower_joint_names", [])
        # gripper mapping/gain
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"].get("gain", 1.0) \
            if "gripper_feedback" in self.config["controller"] else 1.0

        # storage for last messages
        self._follower_joint_state_msg = None
        self._follower_gripper_msg = None

    def set_up_communication(self):
        # Subscriber to follower joint_states
        self.follower_joint_state_sub = self.create_subscription(
            JointState,
            self.follower_joint_state_topic,
            self._follower_joint_state_callback,
            10,
        )

        # Publisher of joint position commands (JointState used as a simple container)
        self.follower_cmd_pub = self.create_publisher(
            JointState,
            self.follower_command_topic,
            10,
        )

        # Optional gripper subscriber
        if self.follower_gripper_state_topic:
            self.follower_gripper_sub = self.create_subscription(
                JointState,
                self.follower_gripper_state_topic,
                self._follower_gripper_callback,
                10,
            )

        # If torque feedback is enabled, wait until joint_states arrives
        if self.enable_torque_feedback:
            t0 = time.time()
            while self._follower_joint_state_msg is None:
                if time.time() - t0 > 5.0:
                    self.get_logger().warning(
                        f"Waiting for follower joint_states on {self.follower_joint_state_topic}..."
                    )
                    t0 = time.time()
                time.sleep(0.05)

    def _follower_joint_state_callback(self, msg: JointState):
        # store the latest follower joint_states
        self._follower_joint_state_msg = msg

    def _follower_gripper_callback(self, msg: JointState):
        self._follower_gripper_msg = msg

    def get_leader_arm_external_joint_torque(self):
        """
        Return a numpy array (num_arm_joints,) of the follower's joint efforts (torques).
        If efforts are not present, return zeros.
        """
        if self._follower_joint_state_msg is None:
            return np.zeros(self.num_arm_joints)

        efforts = list(self._follower_joint_state_msg.effort) if self._follower_joint_state_msg.effort else []
        # Map or slice efforts to expected number of arm joints
        if len(efforts) >= self.num_arm_joints:
            ext_torque = np.array(efforts[: self.num_arm_joints], dtype=float)
        else:
            # pad with zeros if fewer entries
            padded = efforts + [0.0] * (self.num_arm_joints - len(efforts))
            ext_torque = np.array(padded, dtype=float)
        # Re-publish follower efforts for logging/collection as JointState if needed
        return ext_torque

    def get_leader_gripper_feedback(self):
        """
        If a follower gripper topic was provided, return its primary measurement (position or effort).
        Otherwise, try to extract from follower joint_states (assuming last joint is gripper).
        """
        if self._follower_gripper_msg is not None:
            # prefer position if present, else effort, else zero
            if self._follower_gripper_msg.position:
                return float(self._follower_gripper_msg.position[0])
            if self._follower_gripper_msg.effort:
                return float(self._follower_gripper_msg.effort[0])
            return 0.0

        # fallback: try using follower joint_states last element
        if self._follower_joint_state_msg is not None and self._follower_joint_state_msg.position:
            if len(self._follower_joint_state_msg.position) > self.num_arm_joints:
                return float(self._follower_joint_state_msg.position[self.num_arm_joints])
        return 0.0

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        """
        Simple mapping from follower gripper feedback to leader gripper torque.
        Matches the behavior of the Franka example: torque_gripper = -gripper_feedback / gain
        """
        if self.gripper_feedback_gain == 0:
            return 0.0
        torque_gripper = -1.0 * gripper_feedback / self.gripper_feedback_gain
        return float(torque_gripper)

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        """
        Publish leader_arm_pos as a JointState message to follower_command_topic.
        Note: many ros2_control controllers expect different message types (e.g. JointTrajectory or controller-specific).
        If your UR5e controller does not accept JointState as command, write a small adapter node that converts
        this JointState -> trajectory/action/controller command.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # If follower_joint_names defined in YAML use them, otherwise leave empty
        if self.follower_joint_names and len(self.follower_joint_names) >= len(leader_arm_pos):
            msg.name = self.follower_joint_names[: len(leader_arm_pos)]
        else:
            # leave names empty or fill with placeholders if you prefer
            msg.name = []

        # position expects list of floats
        msg.position = list(map(float, leader_arm_pos))
        # Publish arm command
        self.follower_cmd_pub.publish(msg)

        # publish gripper as a separate JointState (single-element) if follower expects it on same topic:
        if self.config.get("communication", {}).get("publish_gripper_on_command_topic", False):
            gmsg = JointState()
            gmsg.header.stamp = self.get_clock().now().to_msg()
            gmsg.position = [float(leader_gripper_pos)]
            self.follower_cmd_pub.publish(gmsg)


def main(args=None):
    rclpy.init(args=args)
    node = FACTRTeleopUR5eROS2()
    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
        node.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()