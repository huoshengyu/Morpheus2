

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import ExternalShutdownException
import time
from gello.robots.robotiq_gripper import RobotiqGripper


class RobotiqSocketController(Node, RobotiqGripper):
    """Controls a Robotiq gripper using gello's Robotiq Gripper interface"""
    
    def __init__(self):
        """Constructor."""
        super().__init__()
        
        # Declare parameters
        self._declare_parameters()
        
        # Create subscriptions for gripper commands and joint states
        self.create_subscription(JointState, self.get_parameter('gripper_command_topic').value, self._gripper_command_callback, 10)
    
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
    
    def _gripper_command_callback(self, msg: JointState) -> None:
        """Handle incoming gripper command messages."""
        try:
            if len(msg.position) == 0 and len(msg.velocity) == 0 and len(msg.effort) == 0:
                return
            self.get_logger().debug(f'Received gripper command: position={msg.position[0]:.3f}m, velocity={msg.velocity[0]:.3f}mm/s, effort={msg.effort[0]:.1f}N')
            self.move(
                position=msg.position[0],
                speed=msg.velocity[0],
                force=msg.effort[0],
            )
        except Exception as e:
            self.get_logger().warn(f'Error processing gripper command: {e}')
    
    
def main(args=None):
    try:
        # Connect to the controller
        rclpy.init(args=args)
        gripper = RobotiqSocketController()
        gripper.connect(hostname="192.168.1.102", port=63352)
        gripper.activate()
        print(gripper.get_current_position())
        gripper.move_and_wait_for_pos(20, 255, 1)
        time.sleep(0.2)
        print(gripper.get_current_position())
        gripper.move_and_wait_for_pos(130, 255, 1)
        time.sleep(0.2)
        print(gripper.get_current_position())
        gripper.move_and_wait_for_pos(20, 255, 1)
        time.sleep(0.2)
        print(gripper.get_current_position())
        
        rclpy.spin(gripper)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == "__main__":
    main()