#! /usr/bin/env python3

# ROS Packages
import rospy

class TeleopBase():
    def __init__(self):
        return NotImplementedError

    def publish(self, rotated_axes, msg):
        return NotImplementedError

    def update(self, msg):
        return NotImplementedError
    
    def loop_once(self):
        return NotImplementedError

    def callback(self, msg):
        return NotImplementedError

    def moveto(self, joint_pos=None):
        return NotImplementedError

if __name__ == '__main__':
    rospy.init_node('teleop_base')

    teleop_base = TeleopBase()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_base.loop_once()
        rate.sleep()
    
    rospy.spin()
