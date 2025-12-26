#! /usr/bin/env python3

import numpy as np
import quaternion
import rospy
import geometry_msgs.msg
import trajectory_msgs.msg

from controller_manager_msgs.srv import SwitchController, ListControllers

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand    

def switch_controller(start_controllers=[], stop_controllers=[], strictness=2, start_asap=False, timeout=5):
    rospy.wait_for_service('controller_manager/switch_controller')
    success = False
    try:
        switch_controller_service = rospy.ServiceProxy(
                            'controller_manager/switch_controller', SwitchController)
        success = switch_controller_service(start_controllers, 
                                            stop_controllers,
                                            strictness,
                                            start_asap,
                                            timeout)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return success
    
def list_controllers():
    rospy.wait_for_service('controller_manager/list_controllers')
    success = False
    try:
        list_controllers_service = rospy.ServiceProxy(
                            'controller_manager/list_controllers', ListControllers)
        success = list_controllers_service()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return success

def publish_joint_pos(publisher=None, joint_names=[], joint_pos=[], duration=5):
    # Publish a position command
    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions = joint_pos
    msg = trajectory_msgs.msg.JointTrajectory()
    msg.joint_names = joint_names
    msg.points = [point]
    publisher.publish(msg)

def command_robotiq2F85(publisher=None, position=0, force=0, speed=0, publish=True):
    # Command Robotiq 2F-85 gripper
    # Inputs:
    #   position        float [0,50]        [0,50] mm
    #   force           float [0,40]        [20,235] N, capped to 40
    #   speed           float [0,150]       [20,150] mm/s

    # Enforce limits
    position = np.clip(position, 0, 1)
    force = np.clip(force, 20, 40)
    speed = np.clip(speed, 20, 150)

    # Write command
    command = Robotiq2FGripper_robot_output()
    command.rACT = 0x1 # Deactivate / Activate {0,1}
    command.rGTO = 0x1 # Stop / "Go To" {0,1}
    command.rATR = 0x0 # Normal / Emergency auto-release {0,1}
    command.rSP = int(255 * (speed - 20) / (150 - 20)) # Minimum / Maximum speed [0,255] (20 to 150 mm/s)
    command.rPR = int(255 * (1 - position)) # Open / Closed position [0,255] (50 to 0 mm)
    command.rFR = int(255 * (force - 20) / (235 - 20)) # Minimum / Maximum force [0,255] (20 to 235 N)
    if publish:
        try:
            publisher.publish(command)
        except ... as e:
            print("Failed to publish Robotiq 2F85 command")
    return command

def command_onrobotRG2FT(publisher=None, position=0, force=0, publish=True):
    # Command OnRobot RG2FT gripper
    # Inputs:
    #   position        float [0,100]       [0,100] mm
    #   force           float [0,40]        [0,40] N 
    #   ### No direct speed control due to hardware constraints, ranges ~[180, 25] mm/s as position increases ###
    #   ### See manual with MODBUS register information below: ###
    #   ### https://onrobot.com/sites/default/files/documents/User_Manual_for_TECHMAN_OMRON_TM_v1.05_EN_0.pdf ###

    # Enforce limits
    position = np.clip(position, 0, 1)
    force = np.clip(force, 0, 40)

    # Make a stop command to interrupt the current motion
    stop = RG2FTCommand()
    stop.Control = 0x0000 # Stop / Grip {0,1} (Gripper completes command before starting next one)
    stop.TargetWidth = 950 # Closed / Open position [0,1000] (0 to 100 mm)
    stop.TargetForce = 0 # Minimum Maximum force [0,400] (0 to 40 N)
    # Make a motion command to move the OnRobot RG2FT gripper
    command = RG2FTCommand()
    command.Control = 0x0001 # Stop / Grip {0,1} (Gripper completes command before starting next one)
    command.TargetWidth = int(1000 * position) # Closed / Open position [0,1000] (0 to 100 mm)
    command.TargetForce = int(10 * force) # Minimum / Maximum force [0,400] (0 to 40 N)
    publisher.publish(stop)
    publisher.publish(command)
    if publish:
        try:
            publisher.publish(command)
        except ... as e:
            print("Failed to publish OnRobot RG2FT command")
    return command

def twist_to_wrench(twist):
    msg = geometry_msgs.msg.Wrench()

    msg.force.x   = twist.linear.x
    msg.force.y   = twist.linear.y
    msg.force.z   = twist.linear.z
    msg.torque.x  = twist.angular.x
    msg.torque.y  = twist.angular.y
    msg.torque.z  = twist.angular.z
    return msg

def add_twist_to_pose(twist, pose, dt):
    # Separate the twist into parts for readability
    v = twist.linear
    w = twist.angular

    # Update pose based on linear velocity and dt
    pose.position.x       += v.x * dt
    pose.position.y       += v.y * dt
    pose.position.z       += v.z * dt
    # Integrate quaternion based on angular velocity and dt
    # https://quaternion.readthedocs.io/en/latest/time_series/#quaternion.quaternion_time_series.integrate_angular_velocity
    # Uses time series or function to find velocities. In this case, only one velocity is given per function call.
    R0 = orientation_to_quaternion(pose.orientation)
    _, R_arr = quaternion.integrate_angular_velocity(lambda _: (w.x, w.y, w.z), 0, dt, R0=R0)
    pose.orientation = quaternion_to_orientation(R_arr[-1]) # Get last

    return pose

def quaternion_to_orientation(quat):
    orientation = geometry_msgs.msg.Quaternion()
    orientation.w = quat.w
    orientation.x = quat.x
    orientation.y = quat.y
    orientation.z = quat.z
    return orientation

def orientation_to_quaternion(orientation):
    quat = np.quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
    return quat

def transform_to_pose(tf):
    msg = geometry_msgs.msg.Pose()

    msg.position.x = tf.translation.x
    msg.position.y = tf.translation.y
    msg.position.z = tf.translation.z
    msg.orientation.x = tf.rotation.x
    msg.orientation.y = tf.rotation.y
    msg.orientation.z = tf.rotation.z
    msg.orientation.w = tf.rotation.w
    return msg