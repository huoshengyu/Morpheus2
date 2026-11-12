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
import subprocess
import threading
import time
from typing import Dict, List, Any

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String, Bool, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DataRecorder(Node):
    """
    Streamlined ROS 2 Data Recorder for manual and bilateral teleoperation experiments.
    Supports customizable topic recording, synchronized CSV export, TF tracking,
    and optional rosbag2 capture.
    """
    def __init__(self, node_name='data_recorder'):
        super().__init__(node_name)

        # Base configuration parameters
        default_base_dir = os.path.expanduser('/root/ros2_ws/data')
        self.declare_parameter('base_dir', default_base_dir)
        self.declare_parameter('basename_topic', '/UCDsession/csv_basename')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('log_rate_hz', 20.0)
        self.declare_parameter('append', False)
        self.declare_parameter('flush_each_row', False)
        self.declare_parameter('record_tf', True)
        self.declare_parameter('record_bag', False)

        # Customizable list of recorded ROS topics
        default_topics = ['/joint_states', '/joy', 'collision/torques']
        self.declare_parameter('recorded_topics', default_topics)

        self.base_dir = self.get_parameter('base_dir').value
        basename_topic = self.get_parameter('basename_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        log_rate_hz = self.get_parameter('log_rate_hz').value
        self.append = self.get_parameter('append').value
        self.flush_each_row = self.get_parameter('flush_each_row').value
        self.record_tf = self.get_parameter('record_tf').value
        self.record_bag = self.get_parameter('record_bag').value
        self.recorded_topics = list(self.get_parameter('recorded_topics').value)

        os.makedirs(self.base_dir, exist_ok=True)

        self.current_basename = "session_log"
        self.is_recording = True
        self.csv_file = None
        self.csv_writer = None
        self.lock = threading.Lock()
        self.bag_process = None

        # TF2 listener
        if self.record_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        # Data cache for recorded topics
        self.data_cache: Dict[str, Any] = {}
        for topic in self.recorded_topics:
            self.data_cache[topic] = None

        # Subscriptions for control signals
        self.basename_sub = self.create_subscription(
            String, basename_topic, self.basename_callback, 10
        )
        self.rec_sub = self.create_subscription(
            Bool, '/session/recording', self.recording_callback, 10
        )

        # Dynamic subscriptions for recorded topics
        self._setup_topic_subscriptions()

        # Logging timer
        timer_period = 1.0 / max(log_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.log_tick)

        self.get_logger().info(
            f"{node_name} initialized. Logging topics: {self.recorded_topics} to: {self.base_dir}"
        )

    def _setup_topic_subscriptions(self):
        """Create subscriptions with type inference based on topic naming conventions."""
        self.subs = []
        for topic in self.recorded_topics:
            clean_topic = topic.strip()
            if not clean_topic:
                continue

            # Determine appropriate message type
            if 'joy' in clean_topic.lower():
                msg_type = Joy
            elif 'torque' in clean_topic.lower() or 'joint' in clean_topic.lower():
                msg_type = JointState
            elif 'twist' in clean_topic.lower():
                msg_type = TwistStamped
            elif 'wrench' in clean_topic.lower():
                msg_type = WrenchStamped
            elif 'pose' in clean_topic.lower() or 'target_frame' in clean_topic.lower():
                msg_type = PoseStamped
            elif 'commands' in clean_topic.lower():
                msg_type = Float64MultiArray
            elif 'distance' in clean_topic.lower():
                msg_type = Float64
            else:
                msg_type = JointState

            sub = self.create_subscription(
                msg_type,
                clean_topic,
                lambda msg, t=clean_topic: self._generic_callback(t, msg),
                10
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to topic '{clean_topic}' as {msg_type.__name__}")

    def _generic_callback(self, topic: str, msg):
        self.data_cache[topic] = msg

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
                header = ['timestamp']
                if self.record_tf:
                    header.extend([
                        'ee_x', 'ee_y', 'ee_z',
                        'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw'
                    ])
                for topic in self.recorded_topics:
                    col_name = topic.strip('/').replace('/', '_')
                    header.append(col_name)

                self.csv_writer.writerow(header)
                self.csv_file.flush()

        # Start rosbag2 recording if requested
        if self.record_bag:
            self._start_bag_recording()

    def _start_bag_recording(self):
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process = None

        bag_path = os.path.join(self.base_dir, f"{self.current_basename}_bag")
        topics_to_bag = ['/tf', '/tf_static'] + self.recorded_topics
        cmd = ['ros2', 'bag', 'record', '-o', bag_path] + topics_to_bag
        try:
            self.bag_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"Started rosbag2 recording to {bag_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag2: {e}")

    def basename_callback(self, msg: String):
        new_name = msg.data.strip()
        if new_name and new_name != self.current_basename:
            self.current_basename = new_name
            self.init_csv_file()

    def recording_callback(self, msg: Bool):
        self.is_recording = msg.data

    def _format_msg_for_csv(self, msg) -> str:
        if msg is None:
            return ""
        if isinstance(msg, JointState):
            pos = ";".join(f"{v:.4f}" for v in msg.position) if msg.position else ""
            eff = ";".join(f"{v:.4f}" for v in msg.effort) if msg.effort else ""
            return f"pos:[{pos}]|eff:[{eff}]"
        elif isinstance(msg, Joy):
            axes = ";".join(f"{v:.4f}" for v in msg.axes) if msg.axes else ""
            btns = ";".join(str(v) for v in msg.buttons) if msg.buttons else ""
            return f"axes:[{axes}]|btns:[{btns}]"
        elif isinstance(msg, PoseStamped):
            p = msg.pose.position
            o = msg.pose.orientation
            return f"{p.x:.4f};{p.y:.4f};{p.z:.4f};{o.x:.4f};{o.y:.4f};{o.z:.4f};{o.w:.4f}"
        elif isinstance(msg, TwistStamped):
            lin = msg.twist.linear
            ang = msg.twist.angular
            return f"{lin.x:.4f};{lin.y:.4f};{lin.z:.4f};{ang.x:.4f};{ang.y:.4f};{ang.z:.4f}"
        elif isinstance(msg, WrenchStamped):
            f = msg.wrench.force
            t = msg.wrench.torque
            return f"{f.x:.4f};{f.y:.4f};{f.z:.4f};{t.x:.4f};{t.y:.4f};{t.z:.4f}"
        elif isinstance(msg, Float64MultiArray):
            return ";".join(f"{v:.4f}" for v in msg.data)
        elif isinstance(msg, Float64):
            return f"{msg.data:.4f}"
        else:
            return str(msg)

    def log_tick(self):
        if not self.is_recording:
            return

        if self.csv_file is None:
            self.init_csv_file()

        now = self.get_clock().now().nanoseconds * 1e-9

        row = [f"{now:.4f}"]

        # Optional TF Lookup
        if self.record_tf and self.tf_buffer:
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
            row.extend([
                f"{ee_pos[0]:.4f}", f"{ee_pos[1]:.4f}", f"{ee_pos[2]:.4f}",
                f"{ee_quat[0]:.4f}", f"{ee_quat[1]:.4f}", f"{ee_quat[2]:.4f}", f"{ee_quat[3]:.4f}"
            ])

        # Formatted data for each configured topic
        for topic in self.recorded_topics:
            msg = self.data_cache.get(topic)
            row.append(self._format_msg_for_csv(msg))

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
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
