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

import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class SessionManager(Node):
    """
    ROS2 Experimental Session Manager
    Publishes session identifiers and recording trigger topics for data logging.
    """
    def __init__(self):
        super().__init__('session_manager')

        self.declare_parameter('subject_id', 'subject_01')
        self.declare_parameter('task_name', 'teleoperation')
        self.declare_parameter('trial', 1)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('basename_topic', '/UCDsession/csv_basename')

        self.subject_id = self.get_parameter('subject_id').value
        self.task_name = self.get_parameter('task_name').value
        self.trial = self.get_parameter('trial').value
        self.auto_start = self.get_parameter('auto_start').value
        basename_topic = self.get_parameter('basename_topic').value

        # Publishers
        self.basename_pub = self.create_publisher(String, basename_topic, 10)
        self.alias_pub = self.create_publisher(String, '/session/basename', 10)
        self.recording_pub = self.create_publisher(Bool, '/session/recording', 10)

        # Periodic status publication (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d")

        self.get_logger().info(
            f"SessionManager initialized: ID={self.subject_id}, Date={self.timestamp}, Task={self.task_name}, Trial={self.trial}"
        )

    def get_basename(self) -> str:
        return f"{self.subject_id}_{self.timestamp}_{self.task_name}_trial{self.trial}"

    def publish_status(self):
        basename_msg = String()
        basename_msg.data = self.get_basename()
        self.basename_pub.publish(basename_msg)
        self.alias_pub.publish(basename_msg)

        rec_msg = Bool()
        rec_msg.data = self.auto_start
        self.recording_pub.publish(rec_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SessionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
