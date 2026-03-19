#! /usr/bin/env python3

# General Imports
import numpy as np  
# ROS Message Imports
from control_msgs.msg import GripperCommand
# Local Imports
from .joy_utils import joy_msg_to_dict

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand  

def joy_msg_to_gripper_command(joy_msg, gripper_type="robotiq"):
    # Convert gamepad button press into open/close command
    # Inputs: 
    #   name            type                limits (Robotiq 2F-85)          limits(OnRobot RG2FT)
    #   position        float [0,50]        [0,50] mm                       [0,100] mm
    #   force           float [0,40]        [20,235] N, capped to 40        [0,40] N
    #   speed           float [0,150]       [20,150] mm/s                   (No speed control, ranges ~[180, 25] mm/s as position increases)
    joy_dict = joy_msg_to_dict(joy_msg, gripper_type)
    
    position = None
    if joy_dict["GRIPPER_CLOSE"] == 1:
        position = 0 # Closed position, proportion
    elif joy_dict["GRIPPER_OPEN"] == 1:
        position = 1 # Open position, proportion
    force = 20 # N
    speed = 100 # mm/s

    if position is not None:
        # Command Robotiq 2F-85 gripper
        if gripper_type == "robotiq":
            gripper_command = get_robotiq2F85_command(position=position, force=force, speed=speed)
        # Command OnRobot RG2FT gripper
        if gripper_type == "onrobot":
            gripper_command = get_onrobotRG2FT_command(position=position, force=force)
        # Command either UR5e gripper type in Gazebo via action server
        if gripper_type == "gazebo":
            gripper_command = GripperCommand()
            gripper_command.position = position
            gripper_command.max_effort = force
    return gripper_command

def get_robotiq2F85_command(position=0, force=0, speed=0, publisher=None):
    # Create command msg for Robotiq 2F-85 gripper
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
    if publisher:
        try:
            publisher.publish(command)
        except Exception as e:
            print("Failed to publish Robotiq 2F85 command: %s"%e)
    return command

def get_onrobotRG2FT_command(position=0, force=0, publisher=None):
    # Create command msg for OnRobot RG2FT gripper
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
    if publisher:
        try:
            publisher.publish(command)
        except Exception as e:
            print("Failed to publish OnRobot RG2FT command: %s"%e)
    return 

def get_onrobotRG2FT_stop(publisher=None):
    # Make a stop command to interrupt the OnRobot RG2FT gripper's current motion
    stop = RG2FTCommand()
    stop.Control = 0x0000 # Stop / Grip {0,1} (Gripper completes command before starting next one)
    stop.TargetWidth = 950 # Closed / Open position [0,1000] (0 to 100 mm)
    stop.TargetForce = 0 # Minimum Maximum force [0,400] (0 to 40 N)
    if publisher:
        try:
            publisher.publish(stop)
        except Exception as e:
            print("Failed to publish OnRobot RG2FT command: %s"%e)
    return stop

def get_trossen_gripper_command(position=0, publisher=None):
    # Create command for Trossen gripper (currently only supports position control)
    # Inputs:
    #   position        float [0,1]        0: Closed position, 1: Open position (Proportional control)
    position = np.clip(position, 0, 1)
    if publisher:
        try:
            publisher.publish(position)
        except Exception as e:
            print("Failed to publish Trossen gripper command: %s"%e)
    return position

def command_trossen_gripper(robot, position=None):
    # Command Trossen gripper (currently only supports position control)
    if position is not None:
        robot.gripper.gripper_controller(robot.gripper.gripper_value*(position * 2 - 1), 0.05)
    return position