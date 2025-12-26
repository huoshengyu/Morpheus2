#! /usr/bin/env python3

# General Packages
import numpy as np
import quaternion
# ROS Packages
import rospy
import tf2_ros
# ROS Messages
import geometry_msgs.msg  
# Local Imports
from utils import twist_to_wrench, add_twist_to_pose, transform_to_pose

class TwistConverter():
    def __init__(self):
        rospy.init_node('converter', anonymous=False)

        # Get params
        self.frame_id = rospy.get_param('~frame_id', default="vx300s/base_link")
        self.end_effector = rospy.get_param('~end_effector', default="vx300s/ee_gripper_link")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate', default=50))

        # Get topics
        self.twist_topic = rospy.get_param("~twist_topic", default="twist_controller/command")
        self.wrench_topic = rospy.get_param("~wrench_topic", default="target_wrench")
        self.pose_topic = rospy.get_param("~pose_topic", default="target_frame")

        # Instantiate subscribers and publishers
        self.twist_sub = rospy.Subscriber(self.twist_topic, geometry_msgs.msg.Twist, self.twist_callback)
        self.wrench_pub = rospy.Publisher(self.wrench_topic, geometry_msgs.msg.WrenchStamped, queue_size=1)
        self.pose_pub = rospy.Publisher(self.pose_topic, geometry_msgs.msg.PoseStamped, queue_size=1)

        # Instantiate tf_buffer and tf_listener to read robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize time for calculating dt
        # Note: rospy.Time.now() returns a rospy.Time instance
        # rospy.get_time() returns float seconds
        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # Instantiate twist, wrench, and pose
        self.twist = geometry_msgs.msg.Twist()
        self.wrench_stamped = geometry_msgs.msg.WrenchStamped()
        self.wrench_stamped.header.stamp        = self.current_time
        self.wrench_stamped.header.frame_id     = self.frame_id
        self.pose_stamped = geometry_msgs.msg.PoseStamped()
        self.pose_stamped.header.stamp          = self.current_time
        self.pose_stamped.header.frame_id       = self.frame_id

        # Initialize target pose to match current pose
        self.initialized = False
        self.initialize_pose()
    
    def set_frame_id(self, frame_id):
        self.frame_id = frame_id

    def set_pose(self, pose):
        # Use a pose msg to directly set the target pose
        if type(pose) == geometry_msgs.msg.Pose:
            self.pose_stamped.pose = pose
        elif type(pose) == geometry_msgs.msg.PoseStamped:
            self.pose_stamped = pose

    def initialize_pose(self):
        # Set (or reset) the target pose to the robot's current pose
        self.initialized = False
        while not self.initialized:
            try:
                self.last_time = self.current_time
                self.current_time = rospy.Time.now()
                tf_stamped = self.tf_buffer.lookup_transform(target_frame=self.frame_id, source_frame=self.end_effector, time=self.current_time, timeout=rospy.Duration(1.0))
            
                self.pose_stamped.header.stamp          = self.current_time
                self.pose_stamped.header.frame_id       = self.frame_id
                self.pose_stamped.pose                  = transform_to_pose(tf_stamped.transform)
                
                self.initialized = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            except Exception as e:
                rospy.logerr(e)
                self.rate.sleep()
                continue

    def update_pose(self):
        # Find dt
        dt = (self.current_time - self.last_time).to_sec()

        # Update pose based on linear velocity and dt
        self.pose_stamped.header.stamp          = self.current_time
        self.pose_stamped.header.frame_id       = self.frame_id
        self.pose_stamped.pose                  = add_twist_to_pose(self.twist, self.pose_stamped.pose, dt)

    def update_wrench(self):
        # Update wrench based on twist
        self.wrench_stamped.header.stamp        = self.current_time
        self.wrench_stamped.header.frame_id     = self.frame_id
        self.wrench_stamped.wrench              = twist_to_wrench(self.twist)

    def twist_callback(self, msg):
        self.twist = msg
        self.last_time = self.current_time
        self.current_time = rospy.Time.now()
        self.update_wrench()
        self.update_pose()

    def publish(self):
        if not rospy.is_shutdown() and self.initialized:
            try:
                self.wrench_pub.publish(self.wrench_stamped)
                self.pose_pub.publish(self.pose_stamped)
            except rospy.ROSInterruptException:
                # Handle 'publish() to closed topic' error.
                # This rarely happens on killing this node.
                pass
            except Exception as e:
                rospy.logwarn(e)

if __name__ == '__main__':
    converter = TwistConverter()
    try:
        while not rospy.is_shutdown():
            converter.publish()
            converter.rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)
