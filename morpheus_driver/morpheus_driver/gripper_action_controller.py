#!/usr/bin/env python3
"""
Real-time generic gripper control with dynamic velocity and effort adjustment.
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

class GripperActionController(Node):
    """Controls an action-based gripper."""
    
    def __init__(self, node_name='gripper_controller', action_name="gripper_controller/gripper_cmd"):
        super().__init__(node_name)
        
        # Action client for gripper control
        self.action_name = action_name
        self._action_client = ActionClient(self, ParallelGripperCommand, self.action_name)
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        
        # State variables
        self.current_position = 0.0
        self.target_position = 0.0
        self.waiting_for_result = False
        
        # Declare parameters
        self._declare_parameters()
        
        # Create subscriptions for gripper commands and joint states
        self.create_subscription(JointState, self.get_parameter('gripper_command_topic').value, self._gripper_command_callback, 10)
        self.create_subscription(JointState, self.get_parameter('joint_state_topic').value, self._joint_state_callback, 10)
        
        # Wait for action server to be available
        self._wait_for_action_server()
    
    def _declare_parameters(self) -> None:
        """Helper to allow child classes to easily change parameter defaults."""
        # Parameters for gripper limits
        self.declare_parameter('open_position', 0.0)            # Defined by gripper hardware interface
        self.declare_parameter('closed_position', 1.0)          # Defined by gripper hardware interface
        self.declare_parameter('max_velocity', 0.150)           # Defined by gripper hardware interface
        self.declare_parameter('max_effort', 40.0)              # Defined by gripper hardware interface
        
        # Parameters for gripper control
        self.declare_parameter('gripper_command_topic', 'gripper_command')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('gripper_joint', 'gripper_joint')
    
    def _wait_for_action_server(self) -> None:
        """Wait for the action server to be available."""
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
        # Calculate target displacement as a proportion of the position range
        displacement_factor = np.clip(self.target_position - self.current_position, -1.0, 1.0)
        
        # Calculate dynamic effort based on state
        effort = self._calculate_effort(displacement_factor, effort_factor)
        velocity = self._calculate_velocity(displacement_factor, velocity_factor)
        
        # Print commands
        self.get_logger().debug(
            f'Moving to {position:.3f} with velocity={velocity:.4f}mm/s ({velocity_factor*100:.0f}%), '
            f'effort={effort:.1f}N ({effort_factor*100:.0f}%)'
        )
        
        send_goal_future = self.move(position, velocity, effort, wait_for_result)
        return send_goal_future
    
    def move(self, position, velocity, effort, wait_for_result=False):
        # Get needed parameters
        open_position = self.get_parameter('open_position').value
        closed_position = self.get_parameter('closed_position').value
        position_range = closed_position - open_position            # Can be negative if open_position > closed_position
        min_position = min(open_position, closed_position)
        max_position = max(open_position, closed_position)
        gripper_joint = self.get_parameter('gripper_joint').value
        
        # Clamp target position
        position = np.clip(position, 0, 1)
        self.target_position = position
        # Scale target position to hardware range
        position_hardware = open_position + position * position_range
        
        # Create the goal
        goal = self.get_goal(name=gripper_joint, position=position_hardware, velocity=velocity, effort=effort)
        
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
            self.get_logger().debug(
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
            self.get_logger().debug(
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
        max_velocity = self.get_parameter('max_velocity').value
        ramp = self._calculate_ramp(displacement_factor)
        velocity = -np.sign(displacement_factor) * max_velocity * ramp * velocity_factor
        return np.clip(velocity, -max_velocity, max_velocity)

    def _calculate_effort(self, displacement_factor, effort_factor):
        """
        Dynamically calculate effort based on displacement factor and effort factor.
        - Lower effort when opening (safety)
        - Higher effort when closing (gripping)
        """
        max_effort = self.get_parameter('max_effort').value
        ramp = self._calculate_ramp(displacement_factor)
        if displacement_factor < 0: ramp = ramp * 0.5  # Reduce effort when opening for safety
        effort = -np.sign(displacement_factor) * max_effort * ramp * effort_factor
        return np.clip(effort, -max_effort, max_effort)
    
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
        self.get_logger().debug('Opening gripper')
        return self.move_to_position(0.0, velocity=velocity, effort=effort, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def close_gripper(self, velocity=None, effort=None, velocity_factor=1.0, effort_factor=1.0, wait_for_result=False):
        """Close gripper with controlled velocity and effort."""
        self.get_logger().debug('Closing gripper')
        return self.move_to_position(1.0, velocity=velocity, effort=effort, velocity_factor=velocity_factor, effort_factor=effort_factor, wait_for_result=wait_for_result)
    
    def grip_object(self, effort=40.0, velocity_factor=1.0, wait_for_result=False):
        """Grip object with specific effort."""
        return self.close_gripper(effort=effort, velocity_factor=velocity_factor, wait_for_result=wait_for_result)
    
    def cancel_goal(self):
        """Cancel current goal."""
        if self._goal_handle:
            self.get_logger().debug('Cancelling goal')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
    
    def _cancel_done_callback(self, future):
        """Handle cancel response."""
        cancel_response = future.result()
        if cancel_response.return_code == 0:  # CancelResponse.ERROR_NONE
            self.get_logger().debug('Goal cancelled successfully')
        else:
            self.get_logger().warn('Failed to cancel goal')
    
    def _gripper_command_callback(self, msg: JointState):
        """Handle incoming gripper command messages."""
        try:
            if len(msg.position) == 0 and len(msg.velocity) == 0 and len(msg.effort) == 0:
                return
            self.get_logger().debug(f'Received gripper command: position={msg.position[0]:.3f}m, velocity={msg.velocity[0]:.3f}mm/s, effort={msg.effort[0]:.1f}N')
            self.move_to_position(
                position=msg.position[0],
                velocity=msg.velocity[0],
                effort=msg.effort[0],
                wait_for_result=False
            )
        except Exception as e:
            self.get_logger().warn(f'Error processing gripper command: {e}')

    def _joint_state_callback(self, msg):
        """Update current position from joint state messages."""
        try:
            gripper_joint = self.get_parameter('gripper_joint').value
            if gripper_joint in msg.name:
                index = msg.name.index(gripper_joint)
                self.current_position = msg.position[index]
                self.get_logger().debug(f'Updated current position from joint state: {self.current_position:.3f}m')
        except Exception as e:
            self.get_logger().warn(f'Error processing joint state message: {e}')


def main(args=None):
    try:
        rclpy.init(args=args)
        
        controller = GripperActionController()
        
        print("\n" + "="*50)
        print("Gripper Control Demo (Action-based)")
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