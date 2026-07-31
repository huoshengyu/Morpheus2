#! /usr/bin/env python3

# General Packages
import numpy as np
# ROS Packages
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Duration, Time
import tf2_ros
# ROS Messages
from geometry_msgs.msg import TwistStamped, WrenchStamped, PoseStamped
# Local Imports
from utils import twist_to_wrench, apply_twist, transform_to_pose

class TwistToPose(Node):
    def __init__(self, node_name='twist_to_pose_node'):
        super().__init__(node_name)

        # Get params
        self.frame_id = self.declare_parameter('frame_id', "base").value
        self.end_effector = self.declare_parameter('end_effector', "tool0").value
        self.rate = self.create_rate(self.declare_parameter('publishing_rate', 125).value)

        # Get topics
        self.twist_topic = self.declare_parameter("twist_topic", "target_twist").value
        self.wrench_topic = self.declare_parameter("wrench_topic", "target_wrench").value
        self.pose_topic = self.declare_parameter("pose_topic", "target_frame").value

        # Initialize subscribers and publishers
        self.twist_sub = self.create_subscription(TwistStamped, self.twist_topic, self.twist_callback, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, self.wrench_topic, 1)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 1)

        # Initialize tf_buffer and tf_listener to read robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Declare twist, tf, pose, and wrench messages
        self.twist_stamped = TwistStamped()
        self.wrench_stamped = WrenchStamped()
        self.pose_stamped = PoseStamped()
    
    def lookup_transform(self, target_frame=None, source_frame=None, time=Time(), timeout=Duration(seconds=1)):
        """ Convenience function to lookup a transform from the tf_buffer with error handling. """
        target_frame = target_frame if target_frame is not None else self.frame_id
        source_frame = source_frame if source_frame is not None else self.end_effector
        try:
            tf_stamped = self.tf_buffer.lookup_transform(target_frame=target_frame, source_frame=source_frame, time=time, timeout=timeout)
            return tf_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Exception in lookup_transform for twist_to_pose: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed lookup_transform for twist_to_pose: {e}")

    def update_pose(self, twist_stamped, tf_stamped):
        # Find dt
        last_time = Time.from_msg(self.pose_stamped.header.stamp)
        current_time = Time.from_msg(twist_stamped.header.stamp)
        dt = (current_time - last_time).nanoseconds * 1e-9 # Convert nanoseconds to seconds
        dt = np.clip(dt, 0.0, 0.1)  # Limit dt to avoid large jumps in pose

        # Update pose based on linear velocity and dt
        self.pose_stamped.header.stamp          = twist_stamped.header.stamp
        self.pose_stamped.header.frame_id       = self.frame_id
        self.pose_stamped.pose                  = apply_twist(twist_stamped.twist, transform_to_pose(tf_stamped.transform), dt=dt)

    def update_wrench(self, twist_stamped):
        # Update wrench based on twist
        self.wrench_stamped.header.stamp        = twist_stamped.header.stamp
        self.wrench_stamped.header.frame_id     = self.frame_id
        self.wrench_stamped.wrench              = twist_to_wrench(twist_stamped.twist, scaling_factor=1)

    def twist_callback(self, msg: TwistStamped):
        try:
            self.twist_stamped = msg
            tf_stamped = self.lookup_transform(
                target_frame = self.frame_id, 
                source_frame = self.end_effector, 
                time=Time.from_msg(msg.header.stamp))
            self.update_pose(msg, tf_stamped)
            self.update_wrench(msg)
            self.publish()
        except Exception as e:
            self.get_logger().warn(f"Exception in twist_callback for twist_to_pose: {e}")

    def publish(self):
        self.wrench_pub.publish(self.wrench_stamped)
        self.pose_pub.publish(self.pose_stamped)
                
def main(args=None):
    try:
        rclpy.init(args=args)
        
        twist_to_pose = TwistToPose()
        
        rclpy.spin(twist_to_pose)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
