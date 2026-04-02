#! /usr/bin/env python3

import numpy as np
import quaternion
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import trajectory_msgs.msg

def publish_joint_pos(publisher=None, joint_names=[], joint_pos=[], duration=5):
    # Publish a position command
    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions = joint_pos
    msg = trajectory_msgs.msg.JointTrajectory()
    msg.joint_names = joint_names
    msg.points = [point]
    publisher.publish(msg)

def twist_to_wrench(twist, scaling_factor=0.1):
    """
    Convert from twist (linear velocity, angular velocity) to wrench (force, torque).
    Primarily a message type conversion, but the scaling factor can be interpreted in units of kg/sec.
    """
    msg = geometry_msgs.msg.Wrench()

    msg.force.x   = twist.linear.x * scaling_factor
    msg.force.y   = twist.linear.y * scaling_factor
    msg.force.z   = twist.linear.z * scaling_factor
    msg.torque.x  = twist.angular.x * scaling_factor
    msg.torque.y  = twist.angular.y * scaling_factor
    msg.torque.z  = twist.angular.z * scaling_factor
    return msg

def add_twist_to_pose(twist, pose, dt=0.1):
    """
    Given twist (linear velocity, angular velocity), pose (position, orientation) and dt (seconds),
    returns the result of moving at rate [twist] for [dt] seconds from the starting [pose].
    """
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