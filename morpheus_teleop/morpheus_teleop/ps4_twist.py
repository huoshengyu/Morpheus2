#! /usr/bin/env python3

# General Imports
import numpy as np
# ROS Imports
import rclpy
from rclpy.executors import ExternalShutdownException

from teleop_twist import TeleopTwist

class TeleopTwistJoy(TeleopTwist):
    def __init__(self):
        super().__init__()

    def msg_to_axes(self, msg, linear_scale=None, angular_scale=None):
        # Convert joy_msg inputs to command outputs in 6 DOF
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        # Set a deadzone and a limit in input values
        for i in range(len(axes)):
            if abs(axes[i]) < self.input_min:
                axes[i] = 0
            if abs(axes[i]) > self.input_max:
                axes[i] = axes[i] / abs(axes[i])

        if linear_scale == None:
            linear_scale = self.linear_scale
        if angular_scale == None:
            angular_scale = self.angular_scale

        # Button remapping so that left joystick/buttons = translation, right joystick/buttons = rotation
        scaled_axes = np.array([ axes[0] * linear_scale, 
                                -axes[1] * linear_scale, 
                                ((buttons[4]) - (axes[2] < 0.0)) * linear_scale, 
                                -axes[4] * angular_scale, 
                                -axes[3] * angular_scale, 
                                -((buttons[5]) - (axes[5] < 0.0)) * angular_scale])

        # Enforce a safety limit on speed
        scaled_axes = np.clip(scaled_axes, -self.output_max, self.output_max)
        
        return scaled_axes

def main(args=None):
    try:
        rclpy.init(args=args)
        
        ps4_twist = TeleopTwistJoy()

        rclpy.spin(ps4_twist)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
