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

# General Packages
import numpy as np
import quaternion
from scipy.spatial.transform import Rotation as R

# ROS Packages
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import sensor_msgs.msg

# Interbotix/Trossen Packages
try:
    from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
except (ImportError, ModuleNotFoundError):
    InterbotixManipulatorXS = None


class TrossenJoy(Node):
    """
    Passes pose commands to the Trossen robot arm 
    as end-effector trajectory commands
    """
    def __init__(self):
        super().__init__('trossen_joy')

        # Parameters
        rate_hz = self.declare_parameter('publishing_rate', 50.0).value
        self.joy_topic = self.declare_parameter('joy_topic', '/joy').value
        self.pose_topic = self.declare_parameter('pose_topic', 'target_frame').value
        self.wrench_topic = self.declare_parameter('wrench_topic', 'target_wrench').value
        self.arm_topic = self.declare_parameter('arm_topic', 'arm_controller/command').value
        self.gripper_topic = self.declare_parameter('gripper_topic', 'gripper_controller/command').value
        robot_model = self.declare_parameter('robot_model', 'vx300s').value

        # Initialize Trossen robot with gripper
        if InterbotixManipulatorXS is not None:
            try:
                self.robot = InterbotixManipulatorXS(robot_model, "arm", "gripper", init_node=False)
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize InterbotixManipulatorXS: {e}. Continuing in mock mode.")
                self.robot = None
        else:
            self.get_logger().info("InterbotixManipulatorXS module not found. Continuing in mock mode.")
            self.robot = None

        # Create subscriber to receive pose commands
        self.pose_sub = self.create_subscription(
            geometry_msgs.msg.PoseStamped, self.pose_topic, self.pose_callback, 10
        )
        self.joy_sub = self.create_subscription(
            sensor_msgs.msg.Joy, self.joy_topic, self.joy_callback, 10
        )

        # Instantiate target pose as attribute
        self.target_pose = None
        self.target_grip = None

        # Timer loop
        timer_period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.loop_once)

    def command_pose(self, pose):
        if pose is None or self.robot is None:
            return
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        translation_vector = np.array([x, y, z]).reshape(-1, 1)
        wx = pose.pose.orientation.x
        wy = pose.pose.orientation.y
        wz = pose.pose.orientation.z
        ww = pose.pose.orientation.w
        quat = quaternion.from_float_array([ww, wx, wy, wz])
        rotation_matrix = quaternion.as_rotation_matrix(quat)
        augmented_matrix = np.concatenate((rotation_matrix, translation_vector), axis=1)
        transformation_matrix = np.concatenate((augmented_matrix, [[0, 0, 0, 1]]), axis=0)
        
        try:
            self.robot.arm.set_ee_pose_matrix(transformation_matrix)
        except Exception as e:
            self.get_logger().debug(f"Error setting ee pose: {e}")

    def command_grip(self, grip):
        if self.robot is None or grip is None:
            return
        try:
            if grip == 1:
                self.robot.gripper.close()
            elif grip == 0:
                self.robot.gripper.open()
        except Exception as e:
            self.get_logger().debug(f"Error commanding gripper: {e}")

    def loop_once(self):
        self.command_pose(self.target_pose)
        self.command_grip(self.target_grip)

    def pose_callback(self, msg):
        self.target_pose = msg
    
    def joy_callback(self, msg):
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.target_grip = 1
        elif len(msg.buttons) > 1 and msg.buttons[1] == 1:
            self.target_grip = 0


def main(args=None):
    rclpy.init(args=args)
    node = TrossenJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
