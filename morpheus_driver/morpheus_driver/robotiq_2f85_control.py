#!/usr/bin/env python3
"""
Real-time Robotiq gripper control with dynamic speed and force adjustment.
This script demonstrates how to:
- Target specific gripper positions
- Dynamically adjust speed based on position
- Dynamically adjust force based on object detection
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand
from rclpy.executors import ExternalShutdownException
import time


class RobotiqGripperController(Node):
    """Controls the Robotiq 2f-85 gripper."""
    
    def __init__(self, action_name="/robotiq_gripper_controller/gripper_cmd"):
        super().__init__('robotiq_gripper_controller')
        
        self.action_name = action_name
        self._action_client = ActionClient(self, ParallelGripperCommand, self.action_name)
        
        # Parameters for gripper control
        self.declare_parameter('max_position', 0.9)         # proportion (fully closed)
        self.declare_parameter('min_position', 0.0)         # proportion (fully open)
        self.declare_parameter('max_speed', 0.150)          # m/s
        self.declare_parameter('max_force', 235.0)          # Newtons
        
        self.max_position = self.get_parameter('max_position').value
        self.min_position = self.get_parameter('min_position').value
        self.position_range = self.max_position - self.min_position
        self.max_speed = self.get_parameter('max_speed').value
        self.max_force = self.get_parameter('max_force').value
        
        self.current_position = 0.0
        self.target_position = 0.0
        self._goal_handle = None
        self.waiting_for_result = False
        
        # Parameters for user commands
        self.declare_parameter('gripper_command_topic', '/gripper_command')
        
        self.gripper_command_topic = self.get_parameter('gripper_command_topic').value
        
        self.create_subscription(ParallelGripperCommand.Goal, self.gripper_command_topic, self._gripper_command_callback, 10)
        
        self.get_logger().info('Robotiq Gripper Controller initialized')
        
        # Wait for action server to be available
        self.get_logger().info(f'Waiting for action server: {self.action_name}')
        self._action_client.wait_for_server()
        self.get_logger().info('Gripper action server is ready!')
    
    def move_to_position(self, target_pos, velocity_factor=1.0, effort_factor=0.1, wait_for_result=False):
        """
        Move gripper to target position with dynamic speed and force via action.
        
        Args:
            target_pos: Target position (0.0 = open, max_position = closed) in meters
            effort_factor: Effort (force) multiplier (0.0 to 1.0)
            velocity_factor: Velocity (speed) multiplier (0.0 to 1.0)
            wait_for_result: Whether to block until goal completes
        
        Returns:
            Goal handle if wait_for_result=False, result otherwise
        """
        # Clamp target position
        target_pos = max(self.min_position, min(self.max_position, target_pos))
        self.target_position = target_pos
        
        # Calculate dynamic speed based on distance
        distance = abs(target_pos - self.current_position)
        distance_factor = min(1.0, distance / self.position_range)  # Ramp up speed
        
        # Calculate dynamic force based on state
        effort = self._calculate_force(distance_factor, effort_factor)
        velocity = self._calculate_speed(distance_factor, velocity_factor)
        
        # Publish commands
        self.get_logger().info(
            f'Moving to {target_pos:.3f}m with velocity={velocity:.4f}m/s ({velocity_factor*100:.0f}%), '
            f'effort={effort:.1f}N ({effort_factor*100:.0f}%)'
        )
        
        # Create the goal
        goal = ParallelGripperCommand.Goal()
        goal.command.name = "robotiq_85_left_knuckle_joint"
        goal.command.position = {target_pos}      # Target position in meters
        goal.command.velocity = {velocity}            # Target velocity in meters/second
        goal.command.effort = {effort}        # Maximum effort (force) in Newtons
        
        # Send the goal
        send_goal_future = self._send_goal_async(goal, wait_for_result)
        self.current_position = target_pos
        return send_goal_future
    
    def _send_goal_async(self, goal, wait_for_result=False):
        """Send goal to action server asynchronously."""
        if self.waiting_for_result:
            self.get_logger().warn('Already waiting for a result, cannot send new goal')
            return
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        if wait_for_result:
            self.waiting_for_result = True
        return send_goal_future
    
    def _goal_response_callback(self, future):
        """Handle goal response from action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by action server')
            self.waiting_for_result = False
            return
        
        self._goal_handle = goal_handle
        self.get_logger().debug('Goal accepted by action server')
        
        # Get the result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)
        return result_future
    
    def _get_result_callback(self, future):
        """Handle action result."""
        result = future.result().result
        self.get_logger().info(
            f'Goal completed | Position reached: {result.state.position[0]:.3f}m | '
            f'Effort applied: {result.state.effort[0]:.1f}N | '
            f'Stalled: {result.stalled} | '
            f'Reached goal: {result.reached_goal}'
        )
        self.waiting_for_result = False
        return result
    
    def _feedback_callback(self, feedback_msg):
        """Handle action feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Feedback: position={feedback.state.position[0]:.3f}m, effort={feedback.state.effort[0]:.1f}N'
        )
        return feedback
    
    def _calculate_speed(self, distance_factor, speed_multiplier):
        """
        Dynamically calculate speed based on distance and multiplier.
        - Slower when close to target (precision)
        - Faster when far (efficiency)
        """
        # Quadratic ramp for smooth motion
        ramp = distance_factor ** 0.5  # Square root for gentler acceleration
        dynamic_speed = self.max_speed * ramp * speed_multiplier
        return max(0.01, min(self.max_speed, dynamic_speed))
    
    def _calculate_force(self, distance_factor, force_multiplier):
        """
        Dynamically calculate force based on position and multiplier.
        - Lower force when approaching (safety)
        - Higher force at end position (gripping)
        """
        # Inverse relationship: high force when gripper closes (low distance)
        if distance_factor > 0.5:
            # Approaching phase: moderate force
            effort_factor_dynamic = 0.4
        elif distance_factor > 0.2:
            # Intermediate phase: ramp up force
            effort_factor_dynamic = 0.6 + 0.2 * (1.0 - distance_factor) / 0.3
        else:
            # Final phase: maximum grip force
            effort_factor_dynamic = 1.0
        
        dynamic_force = self.max_force * effort_factor_dynamic * force_multiplier
        return max(0.0, min(self.max_force, dynamic_force))
    
    def open_gripper(self, velocity_factor=1.0, effort_factor=0.1, wait_for_result=False):
        """Open gripper fully."""
        self.get_logger().info('Opening gripper')
        return self.move_to_position(self.min_position, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def close_gripper(self, velocity_factor=1.0, effort_factor=0.1, wait_for_result=False):
        """Close gripper with controlled force."""
        self.get_logger().info('Closing gripper')
        return self.move_to_position(self.max_position, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def grip_object(self, grip_force=150.0, velocity_factor=1.0,wait_for_result=False):
        """
        Grip object with specific force.
        
        Args:
            grip_force: Target grip force in Newtons
        """
        effort_factor = grip_force / self.max_force
        return self.close_gripper(effort_factor=min(1.0, effort_factor), velocity_factor=velocity_factor,wait_for_result=wait_for_result)
    
    def cancel_goal(self):
        """Cancel current goal."""
        if self._goal_handle:
            self.get_logger().info('Cancelling goal')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
    
    def _cancel_done_callback(self, future):
        """Handle cancel response."""
        cancel_response = future.result()
        if cancel_response.return_code == 0:  # CancelResponse.ERROR_NONE
            self.get_logger().info('Goal cancelled successfully')
        else:
            self.get_logger().warn('Failed to cancel goal')
    
    def _gripper_command_callback(self, msg):
        """Handle incoming gripper command messages."""
        self.get_logger().info(f'Received gripper command: position={msg.position[0]:.3f}m, velocity={msg.velocity[0]:.4f}m/s, effort={msg.effort[0]:.1f}N')
        self.move_to_position(
            target_pos=msg.position[0],
            velocity_factor=msg.velocity[0] / self.max_speed if self.max_speed > 0 else 1.0,
            effort_factor=msg.effort[0] / self.max_force if self.max_force > 0 else 1.0,
            wait_for_result=False
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        
        controller = RobotiqGripperController()
        
        print("\n" + "="*50)
        print("  Robotiq Gripper Control Demo (Action-based)")
        print("="*50 + "\n")
        
        wait_for_result = True
        
        # 1. Open gripper
        print("[1] Opening gripper...")
        goal_future = controller.open_gripper(velocity_factor=1.0, wait_for_result=wait_for_result)
        while controller.waiting_for_result:
            time.sleep(0.1)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 2. Grip with specific effort
        print("\n[2] Gripping object with 25N effort...")
        goal_future = controller.grip_object(grip_force=25.0, velocity_factor=0.6, wait_for_result=wait_for_result)
        while controller.waiting_for_result:
            time.sleep(0.1)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 3. Release
        print("\n[3] Releasing object...")
        goal_future = controller.open_gripper(velocity_factor=0.8, wait_for_result=wait_for_result)
        while controller.waiting_for_result:
            time.sleep(0.1)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 4. Dynamic sequence: approach slowly, grip firmly
        print("\n[4] Approach slowly then grip firmly...")
        goal_future = controller.move_to_position(
            controller.max_position * 0.7,
            effort_factor=0.1,   # Light approach
            velocity_factor=0.3, # Slow approach
            wait_for_result=wait_for_result
        )
        while controller.waiting_for_result:
            time.sleep(0.1)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        goal_future = controller.move_to_position(
            controller.max_position,
            effort_factor=0.2,   # Full grip
            velocity_factor=0.8, # Faster close
            wait_for_result=wait_for_result
        )
        while controller.waiting_for_result:
            time.sleep(0.1)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 5. Release
        print("\n[5] Final release...")
        goal_future = controller.open_gripper(velocity_factor=1.0, wait_for_result=wait_for_result)
        while controller.waiting_for_result:
            time.sleep(0.1)
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