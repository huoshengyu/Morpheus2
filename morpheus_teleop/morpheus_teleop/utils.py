#! /usr/bin/env python3

import numpy as np
import quaternion
import rclpy
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

def apply_twist(twist, pose, dt=0.1):
    """
    Given twist (linear velocity, angular velocity), pose (position, orientation) and dt (seconds),
    returns the result of moving at rate [twist] for [dt] seconds from the starting [pose].
    """
    # Separate the twist into parts for readability
    v = twist.linear
    w = twist.angular

    # Update pose based on linear velocity and dt
    pose.position.x         += v.x * dt
    pose.position.y         += v.y * dt
    pose.position.z         += v.z * dt
    # Update quaternion based on angular velocity and dt
    # https://quaternion.readthedocs.io/en/latest/time_series/#quaternion.quaternion_time_series.integrate_angular_velocity
    q0 = quaternion.quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    qr = quaternion.from_rotation_vector([w.x * dt, w.y * dt, w.z * dt])
    q1 = qr * q0
    pose.orientation.w      = q1.w
    pose.orientation.x      = q1.x
    pose.orientation.y      = q1.y
    pose.orientation.z      = q1.z
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