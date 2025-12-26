#! /usr/bin/env python3

# General Packages
import numpy as np
import quaternion
from scipy.spatial.transform import Rotation as R
# ROS Packages
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
# Interbotix/Trossen Packages
from interbotix_xs_modules.arm import InterbotixManipulatorXS

class TrossenJoy():
    # Passes pose commands to the Trossen robot arm 
    # as end-effector trajectory commands
    def __init__(self):
        rospy.init_node('trossen_joy', anonymous=False)

        # Set loop rate
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate', default=50))

        # Initialize Trossen robot with gripper
        self.robot = InterbotixManipulatorXS("vx300s", "arm", "gripper", init_node=False)
        
        # Get joy topic to get button commands, such as gripper commands
        self.joy_topic = rospy.get_param("~joy_topic", "joy")
        # Get topic for pose and wrench commands from param server, or default
        self.pose_topic = rospy.get_param("~pose_topic", default="target_frame")
        self.wrench_topic = rospy.get_param("~wrench_topic", default="target_wrench")
        # Get topics for Trossen robot commands from param server, or default
        self.arm_topic = rospy.get_param("~arm_topic", default="arm_controller/command")
        self.gripper_topic = rospy.get_param("~gripper_topic", default="gripper_controller/command")

        # Create subscriber to receive pose commands
        self.pose_sub = rospy.Subscriber(self.pose_topic, geometry_msgs.msg.PoseStamped, self.pose_callback)
        self.joy_sub = rospy.Subscriber(self.joy_topic, sensor_msgs.msg.Joy, self.joy_callback)

        # Create publisher to send arm and gripper commands
        #self.arm_pub = rospy.Publisher(self.arm_topic, trajectory_msgs.msg.JointTrajectory, queue_size=1)
        #self.gripper_pub = rospy.Publisher(self.gripper_topic, trajectory_msgs.msg.JointTrajectory, queue_size=1)

        # Instantiate target pose as attribute
        self.target_pose = None
        self.target_grip = None

    def command_pose(self, pose):
        # Convert a geometry_msgs Pose into a command for the Trossen robot
        if pose is None:
            return
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        translation_vector = np.array([x, y, z]).reshape(-1,1)
        wx = pose.pose.orientation.x
        wy = pose.pose.orientation.y
        wz = pose.pose.orientation.z
        ww = pose.pose.orientation.w
        quat = quaternion.from_float_array([ww, wx, wy, wz]) # numpy quaternion uses wxyz order
        rotation_matrix = quaternion.as_rotation_matrix(quat)
        augmented_matrix = np.concatenate((rotation_matrix, translation_vector), axis=1)
        transformation_matrix = np.concatenate((augmented_matrix, [[0, 0, 0, 1]]), axis=0)
        
        self.robot.arm.set_ee_pose_matrix(transformation_matrix)

        #joint_traj_msg = trajectory_msgs.msg.JointTrajectory()
        #self.arm_pub.publish()
        #self.gripper_pub.publish()

    def command_grip(self, grip):
        # Convert a numerical grip value into a command for the Trossen gripper
        if grip == 1:
            self.robot.gripper.close()
        elif grip == 0:
            self.robot.gripper.open()

    def loop_once(self):
        # Command both arm and gripper with the latest received targets
        self.command_pose(self.target_pose)
        self.command_grip(self.target_grip)

    def pose_callback(self, msg):
        # Handle incoming pose message
        self.target_pose = msg
    
    def joy_callback(self, msg):
        # Handle incoming joy message
        if msg.buttons[0] == 1: # A button, close gripper
            self.target_grip = 1
        elif msg.buttons[1] == 1: # B button, open gripper
            self.target_grip = 0

if __name__ == '__main__':
    trossen_joy = TrossenJoy()
    try:
        while not rospy.is_shutdown():
            trossen_joy.loop_once()
            trossen_joy.rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)