#!/usr/bin/env python3
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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import sensor_msgs.msg
import std_msgs.msg
from threading import Lock
from scipy.spatial.transform import Rotation as R


class GoalHaptics(Node):
    """
    Computes distance and orientation error to goal positions,
    publishing rumble feedback to gamepad and cost to topic.
    """
    def __init__(self):
        super().__init__('goal_haptics')

        self.declare_parameter('joy_feedback_topic', '/joy/set_feedback')
        self.declare_parameter('goal_haptics_topic', '/goal/haptics')
        self.declare_parameter('goal_cost_topic', '/goal/cost')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('rate_hz', 20.0)

        joy_feedback_topic = self.get_parameter('joy_feedback_topic').value
        goal_haptics_topic = self.get_parameter('goal_haptics_topic').value
        goal_cost_topic = self.get_parameter('goal_cost_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        rate_hz = self.get_parameter('rate_hz').value

        self.joy_feedback_pub = self.create_publisher(
            sensor_msgs.msg.JoyFeedbackArray, joy_feedback_topic, 10
        )
        self.goal_haptics_pub = self.create_publisher(
            sensor_msgs.msg.JoyFeedbackArray, goal_haptics_topic, 10
        )
        self.goal_cost_pub = self.create_publisher(
            std_msgs.msg.Float64, goal_cost_topic, 10
        )
        self.joy_sub = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.joy_callback, 10
        )

        self.joy_msg = None
        self.joy_msg_mutex = Lock()
        self.on = True
        self.already_toggled = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Default goal: [x, y, z, qx, qy, qz, qw]
        self.goal_pose = [0.35, 0.20, 0.40, 0.0, 0.0, 0.0, 1.0]
        self.goal_weights = [1.0, 1.0, 1.0, 0.2, 0.2, 0.2, 0.2]
        self.cost = None

        timer_period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.loop_once)

        self.get_logger().info("GoalHaptics node initialized")

    def joy_callback(self, msg: sensor_msgs.msg.Joy):
        with self.joy_msg_mutex:
            self.joy_msg = msg

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.ee_frame, Time()
            )
            trans = tf.transform.translation
            rot = tf.transform.rotation
            ee_pose = [trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w]

            trans_dist = np.linalg.norm(np.array(self.goal_pose[:3]) - np.array(ee_pose[:3]))
            r_goal = R.from_quat(self.goal_pose[3:])
            r_ee = R.from_quat(ee_pose[3:])
            rot_dist = (r_goal * r_ee.inv()).magnitude()

            self.cost = float(trans_dist + 0.1 * rot_dist)
        except Exception:
            pass

    def publish(self):
        if self.cost is None:
            return

        cost_msg = std_msgs.msg.Float64()
        cost_msg.data = self.cost
        self.goal_cost_pub.publish(cost_msg)

        feedback_arr = sensor_msgs.msg.JoyFeedbackArray()
        rumble = sensor_msgs.msg.JoyFeedback()
        rumble.type = sensor_msgs.msg.JoyFeedback.TYPE_RUMBLE
        rumble.id = 0

        if not self.on or self.cost < 0.02:
            rumble.intensity = 0.0
        else:
            rumble.intensity = float(np.clip(self.cost * 2.0, 0.0, 1.0))

        feedback_arr.array = [rumble]
        self.joy_feedback_pub.publish(feedback_arr)
        self.goal_haptics_pub.publish(feedback_arr)

    def loop_once(self):
        with self.joy_msg_mutex:
            if self.joy_msg and len(self.joy_msg.buttons) > 6:
                share_pressed = (self.joy_msg.buttons[6] == 1)
                if share_pressed and not self.already_toggled:
                    self.already_toggled = True
                    self.on = not self.on
                    self.get_logger().info(f"Haptic feedback toggled: {'ON' if self.on else 'OFF'}")
                elif not share_pressed:
                    self.already_toggled = False

        self.update()
        self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = GoalHaptics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
