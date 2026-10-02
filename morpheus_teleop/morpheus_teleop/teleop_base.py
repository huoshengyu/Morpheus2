#! /usr/bin/env python3

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