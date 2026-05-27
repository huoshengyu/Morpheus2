#!/usr/bin/env python3
"""
Real-time Robotiq gripper control with dynamic velocity and effort adjustment.
This script demonstrates how to:
- Target specific gripper positions
- Dynamically adjust velocity based on position
- Dynamically adjust effort based on object detection
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand
from rclpy.executors import ExternalShutdownException
import numpy as np
import time


class RobotiqGripperController(Node):
    """Controls the Robotiq 2f-85 gripper."""
    
    def __init__(self, action_name="/robotiq_gripper_controller/gripper_cmd"):
        super().__init__('robotiq_gripper_controller')
        
        self.action_name = action_name
        self._action_client = ActionClient(self, ParallelGripperCommand, self.action_name)
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        
        # Parameters for gripper control
        self.declare_parameter('max_position', 0.9)         # proportion (fully closed)
        self.declare_parameter('min_position', 0.0)         # proportion (fully open)
        self.declare_parameter('max_velocity', 0.150)          # m/s
        self.declare_parameter('max_effort', 235.0)          # Newtons
        
        self.max_position = self.get_parameter('max_position').value
        self.min_position = self.get_parameter('min_position').value
        self.position_range = self.max_position - self.min_position
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_effort = self.get_parameter('max_effort').value
        
        self.current_position = 0.0
        self.target_position = 0.0
        self.waiting_for_result = False
        
        # Parameters for user commands
        self.declare_parameter('gripper_command_topic', 'gripper_command')
        
        self.gripper_command_topic = self.get_parameter('gripper_command_topic').value
        
        self.create_subscription(ParallelGripperCommand.Goal, self.gripper_command_topic, self._gripper_command_callback, 10)
        
        self.get_logger().info('Robotiq Gripper Controller initialized')
        
        # Wait for action server to be available
        self.get_logger().info(f'Waiting for action server: {self.action_name}')
        self._action_client.wait_for_server()
        self.get_logger().info('Gripper action server is ready!')
    
    def move_to_position(self, target_pos, velocity_factor=1.0, effort_factor=0.1, wait_for_result=False):
        """
        Move gripper to target position with dynamic velocity and effort via action.
        
        Args:
            target_pos: Target position (0.0 = open, max_position = closed) in meters
            effort_factor: Effort (force/torque) multiplier (0.0 to 1.0)
            velocity_factor: Velocity (speed) multiplier (0.0 to 1.0)
            wait_for_result: Whether to block until goal completes
        
        Returns:
            Goal handle if wait_for_result=False, result otherwise
        """
        # Clamp target position
        target_pos = max(self.min_position, min(self.max_position, target_pos))
        self.target_position = target_pos
        
        # Calculate dynamic velocity based on distance
        displacement = target_pos - self.current_position
        distance = abs(displacement)
        distance_factor = min(1.0, distance / self.position_range)  # Ramp up velocity
        
        # Calculate dynamic effort based on state
        effort = np.sign(displacement) * self._calculate_effort(distance_factor, effort_factor)
        velocity = np.sign(displacement) * self._calculate_velocity(distance_factor, velocity_factor)
        
        # Publish commands
        self.get_logger().info(
            f'Moving to {target_pos:.3f}m with velocity={velocity:.4f}m/s ({velocity_factor*100:.0f}%), '
            f'effort={effort:.1f}N ({effort_factor*100:.0f}%)'
        )
        
        # Create the goal
        goal = self.get_goal(name="robotiq_85_left_knuckle_joint", position=target_pos, velocity=velocity, effort=effort)
        
        # Send the goal
        send_goal_future = self.send_goal(goal, wait_for_result)
        return send_goal_future
    
    def get_goal(self, name, position, velocity, effort):
        """Helper to create a goal message."""
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [name]
        goal.command.position = [position]
        goal.command.velocity = [velocity]
        goal.command.effort = [effort]
        return goal
    
    def send_goal(self, goal, wait_for_result=False):
        """Send goal to action server asynchronously."""
        if self.waiting_for_result:
            self.get_logger().warn('Already waiting for a result, cannot send new goal')
            return
        if wait_for_result:
            self.waiting_for_result = True
            
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        return self._send_goal_future
    
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
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)
        return self._get_result_future
    
    def _get_result_callback(self, future):
        """Handle action result."""
        result = future.result().result
        try:
            self.get_logger().info(
                f'Goal completed | Position reached: {np.array2string(np.array(result.state.position), precision=3)}m | '
                f'Effort applied: {np.array2string(np.array(result.state.effort), precision=1)}N | '
                f'Stalled: {result.stalled} | '
                f'Reached goal: {result.reached_goal}'
            )
            self.current_position = result.state.position[0]
        except Exception as e:
            self.get_logger().warn(f'Error processing result: {e}')
        self.waiting_for_result = False
        return result
    
    def _feedback_callback(self, feedback_msg):
        """Handle action feedback."""
        feedback = feedback_msg.feedback
        try:
            self.get_logger().info(
                f'Feedback: Position={np.array2string(np.array(feedback.state.position), precision=3)}m | '
                f'Effort={np.array2string(np.array(feedback.state.effort), precision=1)}N'
            )
            self.current_position = feedback.state.position[0]
        except Exception as e:            
            self.get_logger().warn(f'Error processing feedback: {e}')
        return feedback
    
    def _calculate_velocity(self, distance_factor, velocity_factor):
        """
        Dynamically calculate velocity based on distance and velocity factor.
        - Slower when close to target (precision)
        - Faster when far (efficiency)
        """
        # Quadratic ramp for smooth motion
        ramp = distance_factor ** 0.5  # Square root for gentler acceleration
        dynamic_velocity = self.max_velocity * ramp * velocity_factor
        return np.clip(dynamic_velocity, 0.01, self.max_velocity)
    
    def _calculate_effort(self, distance_factor, effort_factor):
        """
        Dynamically calculate effort based on distance and effort factor.
        - Lower effort when approaching (safety)
        - Higher effort at end position (gripping)
        """
        # Inverse relationship: high effort when gripper closes (low distance)
        if distance_factor > 0.5:
            # Approaching phase: moderate effort
            effort_factor_dynamic = 0.4
        elif distance_factor > 0.2:
            # Intermediate phase: ramp up effort
            effort_factor_dynamic = 0.6 + 0.2 * (1.0 - distance_factor) / 0.3
        else:
            # Final phase: maximum grip effort
            effort_factor_dynamic = 1.0
        
        dynamic_effort = self.max_effort * effort_factor_dynamic * effort_factor
        return np.clip(dynamic_effort, 0.0, self.max_effort)

    def open_gripper(self, velocity_factor=1.0, effort_factor=0.1, wait_for_result=False):
        """Open gripper fully."""
        self.get_logger().info('Opening gripper')
        return self.move_to_position(self.min_position, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def close_gripper(self, velocity_factor=1.0, effort_factor=0.1, wait_for_result=False):
        """Close gripper with controlled effort."""
        self.get_logger().info('Closing gripper')
        return self.move_to_position(self.max_position, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def grip_object(self, grip_effort=150.0, velocity_factor=1.0,wait_for_result=False):
        """
        Grip object with specific effort.
        
        Args:
            grip_effort: Target grip effort in Newtons
        """
        effort_factor = grip_effort / self.max_effort
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
        command = msg.command
        try:
            self.get_logger().info(f'Received gripper command: position={command.position[0]:.3f}m, velocity={command.velocity[0]:.4f}m/s, effort={command.effort[0]:.1f}N')
            self.move_to_position(
                target_pos=command.position[0],
                velocity_factor=command.velocity[0] / self.max_velocity if self.max_velocity > 0 else 1.0,
                effort_factor=command.effort[0] / self.max_effort if self.max_effort > 0 else 1.0,
                wait_for_result=False
            )
        except Exception as e:
            self.get_logger().warn(f'Error processing gripper command: {e}')


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
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        # 2. Grip with specific effort
        print("\n[2] Gripping object with 25N effort...")
        goal_future = controller.grip_object(grip_effort=25.0, velocity_factor=0.6, wait_for_result=wait_for_result)
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
            controller.max_position * 0.7,
            effort_factor=0.1,   # Light approach
            velocity_factor=0.3, # Slow approach
            wait_for_result=wait_for_result
        )
        while controller.waiting_for_result:
            time.sleep(0.01)
            rclpy.spin_once(controller)
        time.sleep(0.5)
        
        goal_future = controller.move_to_position(
            controller.max_position,
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
        goal_future = controller.open_gripper(velocity_factor=1.0, wait_for_result=wait_for_result)
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