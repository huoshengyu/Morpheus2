#! /usr/bin/env python3

import rospy
import sensor_msgs.msg

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

keybinds = {
    # 'key':[i,j,k]
    # where i,j select input channel [axes, buttons][i][j]
    # k selects input value
    # Left stick: x = -axes[0], y = axes[1]
    # Right stick: x = -axes[3], y = axes[4]
    # Triggers: LT = axes[2], RT = axes[5]
    # Dpad: L/R = -axes[6], U/D = axes[7]
    # Buttons: [A, B, X, Y, LB, RB, Share, Menu, Xbox, Lstick, Rstick]
    'w':[0,1,-1], # Translate forward
    'a':[0,0,-1], # Translate left
    's':[0,1,1], # Translate backward
    'd':[0,0,1], # Translate right
    'q':[0,2,-1], # Translate down
    'e':[1,4,1], # Translate up
    'i':[0,4,-1], # Pitch up
    'k':[0,4,1], # Pitch down
    'j':[0,3,-1], # Yaw left
    'l':[0,3,1], # Yaw right
    'u':[0,5,-1], # Roll left
    'o':[1,5,1], # Roll right
    '1':[1,0,1], # Close gripper
    '2':[1,1,1], # Open gripper
    '-':[1,8,1], # Swap reference frame (base <--> end effector)
    '=':[1,6,1]  # Toggle goal haptics (goal_haptics.py must be running. Probably only works on Linux anyway)

}

class KeyToJoy():

    def __init__(self):
        self.joy_pub = node.create_publisher(sensor_msgs.msg.Joy, queue_size=5, "/joy")
        self.joy = sensor_msgs.msg.Joy()
        self.joy.axes = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

    def get_key(self):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            key_timeout = 0.1
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], key_timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def key_to_joy(self):
        self.joy = sensor_msgs.msg.Joy()
        self.joy.axes = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        key = self.get_key()
        # If key in binds, send command (only takes one key at a time)
        if key in keybinds.keys():
            if keybinds[key][0] == 0:
                self.joy.axes[keybinds[key][1]] = self.joy.buttons[keybinds[key][1]] + keybinds[key][2]
            elif keybinds[key][0] == 1:
                self.joy.buttons[keybinds[key][1]] = self.joy.buttons[keybinds[key][1]] + keybinds[key][2]
        # If key==esc, exit
        elif len(key) != 0 and ord(key) == 27:
            sys.exit()

        return self.joy
    
    def publish(self):
        self.joy_pub.publish(self.joy)

    def loop_once(self):
        self.key_to_joy()
        self.publish()

if __name__=="__main__":
    rclpy.init()
    node = rclpy.create_node('key_to_joy')

    key_to_joy = KeyToJoy()
    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        key_to_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
