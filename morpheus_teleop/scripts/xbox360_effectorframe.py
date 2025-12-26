#! /usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation as R

from xbox360_twist import TeleopTwistJoy

class TeleopTwistJoyEffectorFrame(TeleopTwistJoy):
    def __init__(self):
        super(TeleopTwistJoyEffectorFrame, self).__init__()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def rotate_axes(self, rearranged_axes, target_frame = "world", source_frame = "tcp_link"):
        # Perform coordinate transfer by rotating to target_frame from source_frame
        rotated_axes = np.array(rearranged_axes)

        effector_offset = R.from_matrix([[ 0,  1,  0],
                                         [ 0,  0,  1],
                                         [-1,  0,  0]])
        
        # Lookup the transform (target_frame) from (source_frame)
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        
        effector_quaternion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

        effector_rotation = R.from_quat(effector_quaternion)

        # Apply the rotation matrix
        offset_axes = np.concatenate((effector_offset.apply(rearranged_axes[:3]), effector_offset.apply(rearranged_axes[3:])))
        rotated_axes = np.concatenate((effector_rotation.apply(offset_axes[:3]), effector_rotation.apply(offset_axes[3:])))
        rotated_axes = np.multiply(rotated_axes, [-1, -1, 1, -1, -1, 1])
        return rotated_axes
    
    def update(self, msg):
        if msg is not None:
            axes = self.msg_to_axes(msg)
            axes = self.rearrange_axes(axes)
            axes = self.rotate_axes(axes)
            return axes
        else:
            return None

if __name__ == '__main__':
    rospy.init_node('xbox360_twist')

    teleop_twist_joy = TeleopTwistJoyEffectorFrame()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
