#! /usr/bin/env python3

import numpy as np
import rospy

from xbox360_effectorframe import TeleopTwistJoyEffectorFrame

class TeleopTwistJoySwapFrame(TeleopTwistJoyEffectorFrame):
    def __init__(self):
        super(TeleopTwistJoySwapFrame, self).__init__()
        self.use_effector_frame = False
        self.already_swapped = False

    def update(self, msg):
        if msg is not None:
            buttons = list(msg.buttons)
            # Check if control frame needs to be swapped
            # buttons[8] = xbox button
            if (buttons[8]==1) and (not self.already_swapped):
                self.already_swapped = True
                self.use_effector_frame = not self.use_effector_frame
            elif (buttons[8]==0):
                self.already_swapped = False

            scaled_axes = self.msg_to_axes(msg)
            rearranged_axes = self.rearrange_axes(scaled_axes)
            rotated_axes = np.array(rearranged_axes)
            if self.use_effector_frame:
                rotated_axes = self.rotate_axes(rearranged_axes)
            return rotated_axes
        else:
            return None

    def loop_once(self):
        msg = self.get_joy_msg()
        rotated_axes = self.update(msg)
        if rotated_axes is not None:
            self.publish(rotated_axes, msg)

if __name__ == '__main__':
    rospy.init_node('xbox360_twist')

    teleop_twist_joy = TeleopTwistJoySwapFrame()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
