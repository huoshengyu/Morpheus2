#! /usr/bin/env python3

# General Packages
import numpy as np
import quaternion
# ROS Packages
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Duration
import tf2_ros
# ROS Messages
import geometry_msgs.msg  
# Local Imports
from utils import twist_to_wrench, add_twist_to_pose, transform_to_pose

class TwistToPose(Node):
    def __init__(self, node_name='twist_to_pose_node'):
        super().__init__(node_name)

        # Get params
        self.frame_id = self.declare_parameter('frame_id', "base").value
        self.end_effector = self.declare_parameter('end_effector', "tool0").value
        self.rate = self.create_rate(self.declare_parameter('publishing_rate', 125).value)

        # Get topics
        self.twist_topic = self.declare_parameter("twist_topic", "twist_controller/command").value
        self.wrench_topic = self.declare_parameter("wrench_topic", "target_wrench").value
        self.pose_topic = self.declare_parameter("pose_topic", "target_frame").value

        # Instantiate subscribers and publishers
        self.twist_sub = self.create_subscription(geometry_msgs.msg.Twist, self.twist_topic, self.twist_callback, 10)
        self.wrench_pub = self.create_publisher(geometry_msgs.msg.WrenchStamped, self.wrench_topic, 1)
        self.pose_pub = self.create_publisher(geometry_msgs.msg.PoseStamped, self.pose_topic, 1)

        # Instantiate tf_buffer and tf_listener to read robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize time for calculating dt
        # Note: rclpy.Time.now() returns a rospy.Time instance
        # rclpy.get_time() returns float seconds
        self.last_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        # Instantiate twist, wrench, and pose
        self.twist = geometry_msgs.msg.Twist()
        self.wrench_stamped = geometry_msgs.msg.WrenchStamped()
        self.wrench_stamped.header.stamp        = self.current_time.to_msg()
        self.wrench_stamped.header.frame_id     = self.frame_id
        self.pose_stamped = geometry_msgs.msg.PoseStamped()
        self.pose_stamped.header.stamp          = self.current_time.to_msg()
        self.pose_stamped.header.frame_id       = self.frame_id

        # Don't forward messages until pose is initialized
        self.initialized = False

    def initialize_pose(self):
        # Set (or reset) the target pose to the robot's current pose
        self.initialized = False
        self.last_time = self.current_time
        self.current_time = self.get_clock().now()
        transform_result = (0, '')
        while not transform_result[0]:
            transform_result = self.tf_buffer.can_transform(target_frame=self.frame_id, source_frame=self.end_effector, time=self.current_time, timeout=Duration(seconds=1), return_debug_tuple=True)
            self.get_logger().info(f"Waiting for transform {self.end_effector} -> {self.frame_id}: {transform_result[1]}")
        while not self.initialized:
            try:
                self.last_time = self.current_time
                self.current_time = self.get_clock().now()
                tf_stamped = self.tf_buffer.lookup_transform(target_frame=self.frame_id, source_frame=self.end_effector, time=self.current_time, timeout=Duration(seconds=1))
            
                self.pose_stamped.header.stamp          = self.current_time.to_msg()
                self.pose_stamped.header.frame_id       = self.frame_id
                self.pose_stamped.pose                  = transform_to_pose(tf_stamped.transform)
                
                self.initialized = True
                self.get_logger().info(f"Initialized pose for twist_to_pose")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().info(f"Exception initializing pose for twist_to_pose: {e}")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize pose for twist_to_pose: {e}")

    def update_pose(self):
        # Find dt
        tf_stamped = self.tf_buffer.lookup_transform(target_frame=self.frame_id, source_frame=self.end_effector, time=self.current_time, timeout=Duration(seconds=1))

        # Update pose based on linear velocity and dt
        self.pose_stamped.header.stamp          = self.current_time.to_msg()
        self.pose_stamped.header.frame_id       = self.frame_id
        self.pose_stamped.pose                  = add_twist_to_pose(self.twist, transform_to_pose(tf_stamped.transform), dt=0.1)

    def update_wrench(self):
        # Update wrench based on twist
        self.wrench_stamped.header.stamp        = self.current_time.to_msg()
        self.wrench_stamped.header.frame_id     = self.frame_id
        self.wrench_stamped.wrench              = twist_to_wrench(self.twist, scaling_factor=1)

    def twist_callback(self, msg):
        if self.initialized:
            try:
                self.twist = msg
                self.last_time = self.current_time
                self.current_time = self.get_clock().now()
                self.update_wrench()
                self.update_pose()
                self.publish()
            except Exception as e:
                self.get_logger().warn(e)
        else:
            self.initialize_pose()

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
