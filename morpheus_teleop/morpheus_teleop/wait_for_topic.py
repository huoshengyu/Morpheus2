#! /usr/bin/env python3

"""Simple node for planning and executing trajectories using MoveIt"""

# ROS Imports
from rclpy.node import Node
from rclpy.task import Future
from rclpy import spin_until_future_complete
# ROS Message Imports
from std_msgs.msg import String

class WaitForTopic(Node):
    def __init__(self, node_name="wait_node", topic=""):
        super().__init__(node_name)

        # Initialize subscriber
        self.topic = topic
        self.subscription = self.create_subscription(String, self.topic, self.topic_callback, 1)
        self.future = Future()
        self.get_logger().info(f"Waiting for topic {self.topic}")
    
    def topic_callback(self, msg):
        self.get_logger().info(f"Received message on topic {self.topic}")
        self.future.set_result(True)

def wait_for_topic(topic, timeout=None):
    wait = WaitForTopic(topic=topic)
    spin_until_future_complete(wait, wait.future, timeout_sec=timeout)
    wait.destroy_node()
