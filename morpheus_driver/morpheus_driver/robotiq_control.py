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
from sensor_msgs.msg import JointState
from rclpy.executors import ExternalShutdownException
import numpy as np
import time

GRIPPER_MIN_POS = 3                                 # Proportion out of 255 (0 = open, 255 = closed)
GRIPPER_MAX_POS = 230                               # Proportion out of 255 (0 = open, 255 = closed)
GRIPPER_MAX_SPEED = 0.150                           # mm/s
GRIPPER_MAX_EFFORT = 235.0                          # N
GRIPPER_RANGE = GRIPPER_MAX_POS - GRIPPER_MIN_POS   # Proportion out of 255 (0 = open, 255 = closed)

class RobotiqGripperController(Node):
    """Controls the Robotiq 2f-85 gripper."""
    
    def __init__(self, action_name="/robotiq_gripper_controller/gripper_cmd"):
        super().__init__('robotiq_gripper_controller')
        
        # Action client for gripper control
        self.action_name = action_name
        self._action_client = ActionClient(self, ParallelGripperCommand, self.action_name)
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        
        # Parameters for gripper safety limits
        self.max_position = self.declare_parameter('max_position', 0.9).value       # proportion (1 = closed)
        self.min_position = self.declare_parameter('min_position', 0.0).value       # proportion (0 = open)
        self.position_range = self.max_position - self.min_position
        self.max_velocity = self.declare_parameter('max_velocity', 0.150).value     # mm/s, [0.0, 0.150]
        self.max_effort = self.declare_parameter('max_effort', 40.0).value          # Newtons, [0.0, 235.0]
        # Sanity check
        assert self.position_range > 0, "max_position must be greater than min_position"
        
        # State variables
        self.current_position = 0.0
        self.target_position = 0.0
        self.waiting_for_result = False
        
        # Parameters for gripper control
        self.gripper_command_topic = self.declare_parameter('gripper_command_topic', 'gripper_command').value
        self.joint_state_topic = self.declare_parameter('joint_state_topic', 'joint_states').value
        self.gripper_joint = self.declare_parameter('gripper_joint', 'gripper_joint').value
        
        self.create_subscription(ParallelGripperCommand.Goal, self.gripper_command_topic, self._gripper_command_callback, 10)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_callback, 10)
        
        self.get_logger().info('Robotiq Gripper Controller initialized')
        
        # Wait for action server to be available
        self.get_logger().info(f'Waiting for action server: {self.action_name}')
        self._action_client.wait_for_server()
        self.get_logger().info('Gripper action server is ready!')
    
    def move_to_position(self, position, velocity=None, effort=None, velocity_factor=1.0, effort_factor=1.0, wait_for_result=False):
        """
        Move gripper to target position with fixed or dynamic velocity and effort via action.
        
        Args:
            position:           Target position as a proportion     [0.0 = open, 1.0 = closed]
            velocity:           Fixed velocity in meters/second     [0.0, 0.150]
            effort:             Fixed effort in Newtons             [0.0, 235.0]
            velocity_factor:    Dynamic velocity gain               [0.0, 1.0]
            effort_factor:      Dynamic effort gain                 [0.0, 1.0]
            wait_for_result:    Whether to block until goal completes
        
        Returns:
            Goal handle if wait_for_result=False, result otherwise
        """
        # Clamp target position
        position = np.clip(position, self.min_position, self.max_position)
        self.target_position = position
        
        # Calculate target displacement as a proportion of the position range
        displacement = position - self.current_position
        displacement_factor = np.clip(displacement / self.position_range, -1.0, 1.0)
        
        # Calculate dynamic effort based on state
        effort = self._calculate_effort(displacement_factor, effort_factor)
        velocity = self._calculate_velocity(displacement_factor, velocity_factor)
        
        # Print commands
        self.get_logger().info(
            f'Moving to {position:.3f} with velocity={velocity:.4f}mm/s ({velocity_factor*100:.0f}%), '
            f'effort={effort:.1f}N ({effort_factor*100:.0f}%)'
        )
        
        # Create the goal
        goal = self.get_goal(name=self.gripper_joint, position=position, velocity=velocity, effort=effort)
        
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
                f'Goal completed | Position: {np.array2string(np.array(result.state.position), precision=3)} | '
                f'Velocity: {np.array2string(np.array(result.state.velocity), precision=3)}mm/s | '
                f'Effort: {np.array2string(np.array(result.state.effort), precision=1)}N | '
                f'Stalled: {result.stalled} | '
                f'Reached goal: {result.reached_goal}'
            )
        except Exception as e:
            self.get_logger().debug(f'Error processing result: {e}')
        self.waiting_for_result = False
        return result
    
    def _feedback_callback(self, feedback_msg):
        """Handle action feedback."""
        feedback = feedback_msg.feedback
        try:
            self.get_logger().info(
                f'Feedback: Position={np.array2string(np.array(feedback.state.position), precision=3)} | '
                f'Velocity={np.array2string(np.array(feedback.state.velocity), precision=3)}mm/s | '
                f'Effort={np.array2string(np.array(feedback.state.effort), precision=1)}N'
            )
        except Exception as e:            
            self.get_logger().debug(f'Error processing feedback: {e}')
        return feedback
    
    def _calculate_velocity(self, displacement_factor, velocity_factor):
        """
        Dynamically calculate velocity based on displacement factor and velocity factor.
        - Slower when close to target (precision)
        - Faster when far (efficiency)
        
        Uses a piecewise linear ramp inspired by flight joysticks:
         Van Baelen et al. (2021). Flying by Feeling: Communicating Flight Envelope Protection through Haptic Feedback. International Journal of Human-Computer Interaction, 37(7), 655-665. https://doi.org/10.1080/10447318.2021.1890489
        """
        ramp = self._calculate_ramp(displacement_factor)
        velocity = np.sign(displacement_factor) * self.max_velocity * ramp * velocity_factor
        return np.clip(velocity, -self.max_velocity, self.max_velocity)
    
    def _calculate_effort(self, displacement_factor, effort_factor):
        """
        Dynamically calculate effort based on displacement factor and effort factor.
        - Lower effort when opening (safety)
        - Higher effort when closing (gripping)
        """
        ramp = self._calculate_ramp(displacement_factor)
        if displacement_factor < 0: ramp = ramp * 0.5  # Reduce effort when opening for safety
        effort = np.sign(displacement_factor) * self.max_effort * ramp * effort_factor
        return np.clip(effort, -self.max_effort, self.max_effort)
    
    def _calculate_ramp(self, displacement_factor):
        """
        Calculate a ramp factor based on displacement for smooth control.
        Uses a piecewise linear ramp inspired by flight joysticks:
        Van Baelen et al.(2021). Flying by Feeling: Communicating Flight Envelope Protection through Haptic Feedback. International Journal of Human-Computer Interaction, 37(7), 655-665. https://doi.org/10.1080/10447318.2021.1890489
        """
        ramp_shallow = abs(displacement_factor) * 0.5       # [0.00, 0.00] to [0.50, 0.25]
        ramp_steep = abs(displacement_factor) * 1.5 - 0.5   # [0.50, 0.25] to [1.00, 1.00]
        return max(ramp_shallow, ramp_steep)

    def open_gripper(self, velocity=None, effort=None, velocity_factor=1.0, effort_factor=1.0, wait_for_result=False):
        """Open gripper with controlled velocity and effort."""
        self.get_logger().info('Opening gripper')
        return self.move_to_position(self.min_position, velocity=velocity, effort=effort, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def close_gripper(self, velocity=None, effort=None, velocity_factor=1.0, effort_factor=1.0, wait_for_result=False):
        """Close gripper with controlled velocity and effort."""
        self.get_logger().info('Closing gripper')
        return self.move_to_position(self.max_position, velocity=velocity, effort=effort, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def grip_object(self, effort=40.0, velocity_factor=1.0, wait_for_result=False):
        """Grip object with specific effort."""
        return self.close_gripper(effort=effort, velocity_factor=velocity_factor, wait_for_result=wait_for_result)
    
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
            self.get_logger().info(f'Received gripper command: position={command.position[0]:.3f}m, velocity={command.velocity[0]:.3f}mm/s, effort={command.effort[0]:.1f}N')
            self.move_to_position(
                target_pos=command.position[0],
                velocity_factor=command.velocity[0] / self.max_velocity if self.max_velocity > 0 else 1.0,
                effort_factor=command.effort[0] / self.max_effort if self.max_effort > 0 else 1.0,
                wait_for_result=False
            )
        except Exception as e:
            self.get_logger().warn(f'Error processing gripper command: {e}')

    def _joint_state_callback(self, msg):
        """Update current position from joint state messages."""
        try:
            if self.gripper_joint in msg.name:
                index = msg.name.index(self.gripper_joint)
                self.current_position = msg.position[index]
                self.get_logger().debug(f'Updated current position from joint state: {self.current_position:.3f}m')
        except Exception as e:
            self.get_logger().warn(f'Error processing joint state message: {e}')


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
            controller.max_position * 0.5,
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
        goal_future = controller.move_to_position(
            controller.max_position * 0.5,
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