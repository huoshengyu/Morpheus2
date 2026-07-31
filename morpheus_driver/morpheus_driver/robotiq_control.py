#!/usr/bin/env python3
"""
Real-time Robotiq gripper control with dynamic velocity and effort adjustment.
This script demonstrates how to:
- Target specific gripper positions
- Dynamically adjust velocity based on position
- Dynamically adjust effort based on object detection
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from morpheus_driver.gripper_action_controller import GripperActionController
import time

class RobotiqGripperController(GripperActionController):
    """Controls the Robotiq 2f-85 gripper."""
    
    def __init__(self, node_name='robotiq_gripper_controller', action_name="robotiq_gripper_controller/gripper_cmd"):
        super().__init__(node_name, action_name)
    
    def _declare_parameters(self):
        """Helper to allow child classes to easily change parameter defaults."""
        # Parameters for gripper limits
        self.declare_parameter('open_position', 0.0)            # proportion (0 = open)
        self.declare_parameter('closed_position', 0.786)          # proportion (1 = closed)
        self.declare_parameter('max_velocity', 0.150)           # mm/s, [0.0, 0.150]
        self.declare_parameter('max_effort', 40.0)              # Newtons, [0.0, 235.0]
        
        # Parameters for gripper control
        self.declare_parameter('gripper_command_topic', 'gripper_command')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('gripper_joint', 'gripper_joint')
    
    def _wait_for_action_server(self):
        """Wait for the action server to be available."""
        self.get_logger().info(f'Waiting for action server: {self.action_name}')
        self._action_client.wait_for_server()
        self.get_logger().info('Robotiq gripper action server is ready!')


def main(args=None):
    try:
        rclpy.init(args=args)
        
        controller = RobotiqGripperController()
        
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()