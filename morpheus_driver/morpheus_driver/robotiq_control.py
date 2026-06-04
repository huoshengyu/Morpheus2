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
        self.declare_parameter('closed_position', 0.9)          # proportion (1 = closed)
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
        
        print("\n" + "="*50)
        print("Robotiq Gripper Control Demo (Action-based)")
        print("="*50 + "\n")
        
        wait_for_result = True
        
        # 1. Open gripper
        print("[1] Opening gripper...")
        goal_future = controller.open_gripper(velocity_factor=1.0, wait_for_result=wait_for_result)
        while controller.waiting_for_result:
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 2. Grip with specific effort
        print("\n[2] Gripping object with 25N effort...")
        goal_future = controller.grip_object(effort=25.0, velocity_factor=0.6, wait_for_result=wait_for_result)
        while controller.waiting_for_result:
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 3. Release
        print("\n[3] Releasing object...")
        goal_future = controller.open_gripper(velocity_factor=0.8, wait_for_result=wait_for_result)
        while controller.waiting_for_result:
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 4. Dynamic sequence: approach slowly, grip firmly
        print("\n[4] Approach slowly then grip firmly...")
        goal_future = controller.move_to_position(
            0.5,
            effort_factor=0.1,   # Light approach
            velocity_factor=0.3, # Slow approach
            wait_for_result=wait_for_result
        )
        while controller.waiting_for_result:
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        goal_future = controller.move_to_position(
            1.0,
            effort_factor=0.2,   # Full grip
            velocity_factor=0.8, # Faster close
            wait_for_result=wait_for_result
        )
        while controller.waiting_for_result:
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 5. Release
        print("\n[5] Final release...")
        goal_future = controller.move_to_position(
            0.5,
            effort_factor=0.1,   # Light approach
            velocity_factor=0.3, # Slow approach
            wait_for_result=wait_for_result
        )
        while controller.waiting_for_result:
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)

        print("\n" + "="*50)
        print("  Demo Complete!")
        print("="*50 + "\n")
        
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()