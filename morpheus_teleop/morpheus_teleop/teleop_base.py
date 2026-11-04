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


"""
Base class for teleoperation

Receives inputs from controller drivers.
Converts inputs to outputs.
Sends outputs to robot drivers.
"""

# ROS Packages
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class TeleopBase(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

    def publish(self, rotated_axes, msg):
        return NotImplementedError

    def update(self, msg):
        return NotImplementedError

def main(args=None):
    try:
        rclpy.init(args=args)
        
        teleop_base = TeleopBase()

        rclpy.spin(teleop_base)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()