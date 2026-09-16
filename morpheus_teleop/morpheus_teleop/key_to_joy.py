#! /usr/bin/env python3

# General Imports
import sys
import numpy as np
import sensor_msgs.msg
# Keyboard Input Imports
from pynput import keyboard
# ROS Imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# ROS2 Joy controller button mappings
# https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md
# 'key':[i,j] where i,j select input channel [buttons, axes][i][j]
# Buttons:      [X/A, O/B, S/X, T/Y, Share, Xbox/PS, Menu, Lstick, Rstick, LB, RB, Dpad U, D, L, R]
# Axes:         [-Lstick x, Lstick y, -Rstick x, Rstick y, LT, RT]
button_mapping = {
    # Keyboard Input    Gamepad Input       UR5e Output         Interbotix Output   
    # Buttons:  [X/A, O/B, S/X, T/Y, Share, Xbox/PS, Menu, Lstick, Rstick, LB, RB, Dpad U, D, L, R]
    '1':[0, 0, 1],      # X/Down            Close gripper       Gripper PWM decrease
    '2':[0, 1, 1],      # O/Right           Open gripper        Open gripper
    '3':[0, 2, 1],      # S/Left                                Close gripper
    '4':[0, 3, 1],      # T/Up                                  Gripper PWM increase
    '0':[0, 4, 1],      # Share             Controller haptics  Sleep pose
    '=':[0, 5, 1],      # PS4               Frame change        Torque enable
    '-':[0, 6, 1],      # Menu              Home pose           Home pose
    'x':[0, 7, 1],      # Lstick press      Prev waypoint       Prev waypoint
    ',':[0, 8, 1],      # Rstick press      Next waypoint       Next waypoint
    'e':[0, 9, 1],      # LB                Translate up        Pan left
    'u':[0,10, 1],      # RB                Roll right          Pan right
    't':[0,11, 1],      # Dpad up                               Speed up
    'g':[0,12, 1],      # Dpad down                             Speed down
    'f':[0,13, 1],      # Dpad left                             Speed coarse
    'h':[0,14, 1],      # Dpad right                            Speed fine
    # Axes:     [-Lstick x, Lstick y, -Rstick x, Rstick y, LT, RT]
    'a':[1, 0, 1],      # Lstick left       Translate left      Translate forward
    'd':[1, 0,-1],      # Lstick right      Translate right     Translate back
    'w':[1, 1, 1],      # Lstick up         Translate forward   Translate up
    's':[1, 1,-1],      # Lstick down       Translate backward  Translate down
    'j':[1, 2, 1],      # Rstick left       Yaw left            Roll left
    'l':[1, 2,-1],      # Rstick right      Yaw right           Roll right
    'i':[1, 3, 1],      # Rstick up         Pitch up            Pitch up
    'k':[1, 3,-1],      # Rstick down       Pitch down          Pitch down
    'q':[1, 4, 1],      # LT                Translate down      Rotate left
    'o':[1, 5, 1],      # RT                Roll left           Rotate right
}

class KeyToJoy(Node):
    def __init__(self):
        super().__init__('key_to_joy')

        self.joy_pub = self.create_publisher(sensor_msgs.msg.Joy, "joy", 5)
        self.joy = sensor_msgs.msg.Joy()

        # Set controller type
        controller = self.declare_parameter("controller", "ps4").value
        self.keybinds = button_mapping
        self.get_logger().info("Key to Joy using controller " + controller + ".")

        # Keep a record of what keys should be when not pressed
        if controller == "xbox360":
            self._default_axes = np.array([0.0,0.0,0.0,0.0,1.0,1.0,0.0,0.0]) # Triggers should be 1 when unpressed
        else:
            self._default_axes = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]) # Triggers should be 0 when unpressed
        self._default_buttons = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]) # Need 21 buttons to accomodate arbitrary controllers
        # Initialize joystick values
        self.joy.axes = np.copy(self._default_axes)
        self.joy.buttons = np.copy(self._default_buttons)

        # Create keyboard listener object
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
        
        # Create timer to publish at a fixed rate
        timer_rate = 125 # Xbox and PS4 controller polling rate is 125 Hz
        timer_period = 1.0 / timer_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def on_press(self, key):
        # If key in binds, send command value (only takes one key at a time)
        # Note that instance is checked first to avoid requesting nonexistent attributes
        if isinstance(key, keyboard.KeyCode) and key.char in self.keybinds.keys():
            k = key.char
            if self.keybinds[k][0] == 0:
                self.joy.buttons[self.keybinds[k][1]] = self.keybinds[k][2]
            elif self.keybinds[k][0] == 1:
                self.joy.axes[self.keybinds[k][1]] = self.keybinds[k][2]
        elif isinstance(key, keyboard.Key) and key == keyboard.Key.esc: # If key==esc, exit
            sys.exit()
    
    def on_release(self, key):
        # If key in binds, reset to default value (only takes one key at a time)
        # Note that instance is checked first to avoid requesting nonexistent attributes
        if isinstance(key, keyboard.KeyCode) and key.char in self.keybinds.keys():
            k = key.char
            if self.keybinds[k][0] == 0:
                self.joy.buttons[self.keybinds[k][1]] = self._default_buttons[self.keybinds[k][1]]
            elif self.keybinds[k][0] == 1:
                self.joy.axes[self.keybinds[k][1]] = self._default_axes[self.keybinds[k][1]]
    
    def publish(self):
        self.joy_pub.publish(self.joy)
    
    def timer_callback(self):
        self.publish()

def main(args=None):
    try:
        rclpy.init(args=args)
            
        key_to_joy = KeyToJoy()

        rclpy.spin(key_to_joy)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()