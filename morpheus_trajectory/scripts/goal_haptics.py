#! /usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
import sensor_msgs.msg
import std_msgs.msg
from threading import Lock
from collections import deque
from scipy.spatial.transform import Rotation as R

class GoalHaptics():
    def __init__(self):
        joy_feedback_topic = rospy.get_param("~joy_feedback_topic", "/joy/set_feedback")
        goal_haptics_topic = rospy.get_param("~goal_haptics_topic", "/goal/haptics")
        goal_cost_topic = rospy.get_param("~goal_haptics_cost_topic", "/goal/cost")

        self.joy_feedback_pub = rospy.Publisher(joy_feedback_topic, sensor_msgs.msg.JoyFeedbackArray, queue_size=0)
        self.goal_haptics_pub = rospy.Publisher(goal_haptics_topic, sensor_msgs.msg.JoyFeedbackArray, queue_size=0)
        self.goal_cost_pub = rospy.Publisher(goal_cost_topic, std_msgs.msg.Float64, queue_size=0)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)

        self.joy_msg = None
        self.joy_msg_mutex = Lock()

        self.on = False
        self.already_toggled = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.goal_pose_list = [] # (x, y, z, Xr, Yr, Zr, W)

        self.goal_weight_list = []
        self.cost = None

        # Attempt to retrieve ideal goal pose from parameter server
        self.goal_dict = rospy.get_param("/goal", None)

        # Attempt to retrieve a list of goal presets from parameter server
        self.goal_name_list = rospy.get_param("/goal/name_vector", None)
        if self.goal_name_list is not None:
            rospy.loginfo("Using goal name list from parameter server")
        else:
            self.goal_name_list = ["keyhole"]
            rospy.loginfo("Using default goal name list")

        # Retrieve details for all named poses
        for name in self.goal_name_list:
            goal_scene = rospy.get_param("/goal/" + name, None)
            if goal_scene is not None:
                rospy.loginfo("Using goal transform from parameter server")
                # Retrieve translation and quaternion from the goal transform dict
                goal_pose = goal_scene['goal_pose']
                self.goal_pose_list.append([goal_pose['position']['x'], 
                                        goal_pose['position']['y'], 
                                        goal_pose['position']['z'],
                                        goal_pose['orientation']['x'], 
                                        goal_pose['orientation']['y'], 
                                        goal_pose['orientation']['z'], 
                                        goal_pose['orientation']['w']])
            else:
                rospy.loginfo("Failed to retrieve goal transform")

            # Attempt to retrieve goal pose weights from parameter server    
            if self.goal_scene is not None:
                rospy.loginfo("Using goal weights from parameter server")
                # Retrieve translation and quaternion from the goal transform dict
                self.goal_weight_list.append([goal_scene['position_weights']['x'], 
                                        goal_scene['position_weights']['y'], 
                                        goal_scene['position_weights']['z'],
                                        goal_scene['orientation_weights']['x'], 
                                        goal_scene['orientation_weights']['y'], 
                                        goal_scene['orientation_weights']['z'], 
                                        goal_scene['orientation_weights']['w']])
            else:
                rospy.loginfo("Failed to retrieve goal weights")

        self.force_feedback_msg = None
        self.force_feedback_msg_mutex = Lock()

        self.force_feedback_filter_size = 5  # Adjust this value for the moving average window size
        self.force_feedback_buffer = deque(maxlen=self.force_feedback_filter_size)


    def update(self):
        transform = None
        while transform == None:
            try:
                transform = self.tf_buffer.lookup_transform("world", "tcp_link", rospy.Time(0), rospy.Duration(5.0))
            except:
                rospy.loginfo("Waiting for tcp_link transform")
        end_effector_pose = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
                             transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        
        cost_list = [self.get_cost(end_effector_pose, self.goal_pose_list[i], self.goal_weight_list[i]) for i in range(len(self.goal_pose_list))]
        
        # Calculate a cost for the current pose, as a function of distance to goal in translation and rotation
        self.cost = min(cost_list)

    def publish(self):
        if self.cost is not None:
            # Clear force feedback message and make new array
            self.joy_feedback_array_msg = sensor_msgs.msg.JoyFeedbackArray()
            joy_feedback_array = []

            # Make new force feedback message and add to array
            joy_feedback_msg = sensor_msgs.msg.JoyFeedback()
            joy_feedback_msg.type = 1 # Rumble
            joy_feedback_msg.id = 0
            # If cost is below threshold, set feedback intensity to 0
            rospy.loginfo("Cost was " + str(self.cost))
            if self.cost < 0.01 or not self.on:
                joy_feedback_msg.intensity = 0.0
            # Else, make sure intensity does not exceed 1.0
            else:
                joy_feedback_msg.intensity = min(1.0, self.cost)
            joy_feedback_array.append(joy_feedback_msg)

            # Make new LED feedback message and add to array (Current controller does not have LED)
            led_feedback_msg = sensor_msgs.msg.JoyFeedback()
            led_feedback_msg.type = 0 # LED
            led_feedback_msg.id = 0
            # If cost is below threshold, set feedback intensity to 0
            if self.cost < 0.01 or not self.on:
                led_feedback_msg.intensity = 0.0
            # Else, make sure intensity does not exceed 1.0
            else:
                led_feedback_msg.intensity = min(1.0, self.cost)
            joy_feedback_array.append(led_feedback_msg)

            # Assign new array to the joy_feedback_array_msg
            self.joy_feedback_array_msg.array = joy_feedback_array

            # Publish message to joy feedback topic
            self.joy_feedback_pub.publish(self.joy_feedback_array_msg)
            # Publish message to goal haptics topic (change message if non-joystick haptic device needs to convey different info)
            self.goal_haptics_pub.publish(self.joy_feedback_array_msg)
            self.goal_cost_pub.publish(self.cost)

    def get_cost(self, end_effector_pose, goal_pose, weights = [1, 1, 1, 1, 1, 1, 1]):
        # Calculate final - initial translation
        translation_to_goal = np.subtract(goal_pose[:3], end_effector_pose[:3])

        # Calculate final - initial rotation
        rotation_to_goal = R.concatenate((R.from_quat(goal_pose[3:]), R.from_quat(end_effector_pose[3:]).inv())).as_quat()
        
        # Calculate a cost for the current pose, as a function of distance to goal in translation and rotation
        cost = (np.sum(np.multiply(weights[:3], translation_to_goal)) + 
                np.sum(np.multiply(weights[3:], rotation_to_goal)))
        return cost

    def loop_once(self):
        if self.joy_msg is not None:
            buttons = list(self.joy_msg.buttons)
            # Check if control frame needs to be swapped
            # buttons[6] = share button
            if (buttons[6]==1) and (not self.already_toggled):
                self.already_toggled = True
                self.on = not self.on
            elif (buttons[6]==0):
                self.already_toggled = False
        self.update()
        self.publish()
    
    def callback(self, msg):
        # Retrieve joy_msg
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()

if __name__ == '__main__':
    rospy.init_node('goal_haptics')

    goal_haptics = GoalHaptics()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        goal_haptics.loop_once()
        rate.sleep()
    
    rospy.spin()
