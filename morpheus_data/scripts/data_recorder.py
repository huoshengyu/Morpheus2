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

import csv
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String, Bool
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DataRecorder(Node):
    """
    ROS2 Data Recorder for manual teleoperation experiments.
    Subscribes to robot joint states, teleoperation inputs, TF poses, and collision torques,
    logging synchronized rows to a CSV file.
    """
    def __init__(self, node_name='data_recorder'):
        super().__init__(node_name)

        # Declare parameters
        default_base_dir = os.path.expanduser('~/ros2_ws/data')
        self.declare_parameter('base_dir', default_base_dir)
        self.declare_parameter('basename_topic', '/UCDsession/csv_basename')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('log_rate_hz', 20.0)
        self.declare_parameter('append', False)
        self.declare_parameter('flush_each_row', False)

        self.base_dir = self.get_parameter('base_dir').value
        basename_topic = self.get_parameter('basename_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        log_rate_hz = self.get_parameter('log_rate_hz').value
        self.append = self.get_parameter('append').value
        self.flush_each_row = self.get_parameter('flush_each_row').value

        os.makedirs(self.base_dir, exist_ok=True)

        self.current_basename = "session_log"
        self.is_recording = True
        self.csv_file = None
        self.csv_writer = None
        self.lock = threading.Lock()

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State caches
        self.last_joint_state = None
        self.last_joy = None
        self.last_collision_torques = None

        # Subscribers
        self.basename_sub = self.create_subscription(
            String, basename_topic, self.basename_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        self.collision_sub = self.create_subscription(
            JointState, 'collision/torques', self.collision_callback, 10
        )
        self.rec_sub = self.create_subscription(
            Bool, '/session/recording', self.recording_callback, 10
        )

        # Logging timer
        timer_period = 1.0 / max(log_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.log_tick)

        self.get_logger().info(f"{node_name} initialized. Logging to: {self.base_dir}")

    def init_csv_file(self):
        with self.lock:
            if self.csv_file:
                self.csv_file.close()
            csv_path = os.path.join(self.base_dir, f"{self.current_basename}.csv")
            file_exists = os.path.isfile(csv_path)
            mode = 'a' if self.append and file_exists else 'w'
            self.csv_file = open(csv_path, mode=mode, newline='')
            self.csv_writer = csv.writer(self.csv_file)
            if mode == 'w':
                header = [
                    'timestamp', 'ee_x', 'ee_y', 'ee_z',
                    'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw',
                    'joints_pos', 'joints_vel', 'joints_effort',
                    'joy_axes', 'joy_buttons',
                    'collision_torques'
                ]
                self.csv_writer.writerow(header)
                self.csv_file.flush()

    def basename_callback(self, msg: String):
        new_name = msg.data.strip()
        if new_name and new_name != self.current_basename:
            self.current_basename = new_name
            self.init_csv_file()

    def recording_callback(self, msg: Bool):
        self.is_recording = msg.data

    def joint_callback(self, msg: JointState):
        self.last_joint_state = msg

    def joy_callback(self, msg: Joy):
        self.last_joy = msg

    def collision_callback(self, msg: JointState):
        self.last_collision_torques = msg

    def log_tick(self):
        if not self.is_recording:
            return

        if self.csv_file is None:
            self.init_csv_file()

        now = self.get_clock().now().nanoseconds * 1e-9

        # EE pose lookup
        ee_pos = [0.0, 0.0, 0.0]
        ee_quat = [0.0, 0.0, 0.0, 1.0]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.ee_frame, Time()
            )
            trans = tf.transform.translation
            rot = tf.transform.rotation
            ee_pos = [trans.x, trans.y, trans.z]
            ee_quat = [rot.x, rot.y, rot.z, rot.w]
        except Exception:
            pass

        joints_pos = list(self.last_joint_state.position) if self.last_joint_state else []
        joints_vel = list(self.last_joint_state.velocity) if self.last_joint_state else []
        joints_eff = list(self.last_joint_state.effort) if self.last_joint_state else []

        joy_axes = list(self.last_joy.axes) if self.last_joy else []
        joy_buttons = list(self.last_joy.buttons) if self.last_joy else []

        coll_torques = list(self.last_collision_torques.effort) if self.last_collision_torques else []

        row = [
            f"{now:.4f}",
            f"{ee_pos[0]:.4f}", f"{ee_pos[1]:.4f}", f"{ee_pos[2]:.4f}",
            f"{ee_quat[0]:.4f}", f"{ee_quat[1]:.4f}", f"{ee_quat[2]:.4f}", f"{ee_quat[3]:.4f}",
            ";".join(f"{v:.4f}" for v in joints_pos),
            ";".join(f"{v:.4f}" for v in joints_vel),
            ";".join(f"{v:.4f}" for v in joints_eff),
            ";".join(f"{v:.4f}" for v in joy_axes),
            ";".join(str(v) for v in joy_buttons),
            ";".join(f"{v:.4f}" for v in coll_torques),
        ]

        with self.lock:
            if self.csv_writer:
                self.csv_writer.writerow(row)
                if self.flush_each_row:
                    self.csv_file.flush()

    def destroy_node(self):
        with self.lock:
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
