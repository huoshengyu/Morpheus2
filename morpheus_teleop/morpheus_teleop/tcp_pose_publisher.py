#!/usr/bin/env python3

import rclpy
import tf2_ros
import geometry_msgs.msg

def transform_callback(tf_buffer):
    try:
        # Lookup the transform to (target_frame) from (source_frame)
        transform = tf_buffer.lookup_transform("world", "tcp_link", rospy.Time(0), rospy.Duration(1.0))
        
        # Create a Pose message
        pose_msg = geometry_msgs.msg.Pose()
        pose_msg.position.x = transform.transform.translation.x
        pose_msg.position.y = transform.transform.translation.y
        pose_msg.position.z = transform.transform.translation.z
        pose_msg.orientation.x = transform.transform.rotation.x
        pose_msg.orientation.y = transform.transform.rotation.y
        pose_msg.orientation.z = transform.transform.rotation.z
        pose_msg.orientation.w = transform.transform.rotation.w
        
        # Publish the Pose message
        pose_pub.publish(pose_msg)
        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        self.get_logger().warn("Transform lookup failed!")

if __name__ == '__main__':
    with rclpy.init(args=sys.argv):
    node = rclpy.create_node('transform_to_pose_node')
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    pose_pub = self.create_publisher(geometry_msgs.msg.Pose, '/tcp_pose', 10)
    
    rate = rospy.Rate(10)  # hz
    
    while not rospy.is_shutdown():
        transform_callback(tf_buffer)
        rate.sleep()
