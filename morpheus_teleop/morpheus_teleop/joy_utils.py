

#! /usr/bin/env python3

# General Imports
import numpy as np
from copy import deepcopy
# ROS Imports
from rclpy.logging import get_logger

# ROS2 Joy controller button mappings
# https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md
# 'key':[i,j] where i,j select input channel [buttons, axes][i][j]
# Buttons:      [X/A, O/B, S/X, T/Y, Share, Xbox/PS, Menu, Lstick, Rstick, LB, RB, Dpad U, D, L, R]
# Axes:         [-Lstick x, Lstick y, -Rstick x, Rstick y, LT, RT]
bm = {
    # Command           Gamepad Input
    # Buttons:  [X/A, O/B, S/X, T/Y, Share, Xbox/PS, Menu, Lstick, Rstick, LB, RB, Dpad U, D, L, R]
    "GRIPPER_PWM_DEC":  (0, 0),
    "GRIPPER_OPEN":     (0, 1),
    "GRIPPER_PWM_INC":  (0, 2),
    "GRIPPER_CLOSE":    (0, 3),
    "SLEEP_POSE":       (0, 4),
    "TORQUE_ENABLE":    (0, 5),
    "HOME_POSE":        (0, 6),
    "FLIP_EE_X":        (0, 7),
    "FLIP_EE_ROLL":     (0, 8),
    "WAIST_CCW":        (0, 9),
    "WAIST_CW":         (0, 10),
    "SPEED_INC":        (0, 11),
    "SPEED_DEC":        (0, 12),
    "SPEED_COARSE":     (0, 13),
    "SPEED_FINE":       (0, 14),
    # Axes:     [-Lstick x, Lstick y, -Rstick x, Rstick y, LT, RT]
    "EE_X":             (1, 0),
    "EE_Z":             (1, 1),
    "EE_ROLL":          (1, 2),
    "EE_PITCH":         (1, 3),
    "EE_Y_INC":         (1, 4),
    "EE_Y_DEC":         (1, 5),
    }

def get_joy_msg(joy_msg, joy_msg_mutex):
    """Retrieve joy_msg and return a safe copy"""
    joy_msg_mutex.acquire()
    msg = deepcopy(joy_msg)
    joy_msg_mutex.release()
    return msg

def joy_msg_to_dict(msg, linear_scale=1.0, angular_scale=0.1, input_min=0.05, input_max=1.0, output_max=1.0, logger=get_logger("joy_utils")):
    """Convert joy_msg inputs to command outputs"""
    # Get button and axis values from joy_msg
    buttons = np.array(msg.buttons)
    axes = np.array(msg.axes)

    # Set a deadzone and a limit in input values
    for i in range(len(axes)):
        if abs(axes[i]) < input_min:
            axes[i] = 0
        if abs(axes[i]) > input_max:
            axes[i] = np.clip(axes[i], -input_max, input_max)

    # Calculate joy_dict values based on button and axis inputs
    padded_buttons = np.append(buttons, [0] * (21 - len(buttons))) # Pad buttons to max button length (21)
    padded_axes = np.append(axes, [0] * (21 - len(axes))) # Pad axes to match max button length
    cmd = np.array([padded_buttons, padded_axes])
    joy_dict = {}
    try:
        # Retrieve movement controls by name
        for key, value in bm.items():
            joy_dict[key] = cmd[value]
        # Apply motion scaling factors
        joy_dict["EE_X"]        *= linear_scale
        joy_dict["EE_Y_INC"]    *= linear_scale
        joy_dict["EE_Y_DEC"]    *= linear_scale
        joy_dict["EE_Z"]        *= linear_scale
        joy_dict["EE_ROLL"]     *= angular_scale
        joy_dict["EE_PITCH"]    *= angular_scale
        joy_dict["WAIST_CCW"]   *= angular_scale
        joy_dict["WAIST_CW"]    *= angular_scale
        # Combine controls to get final outputs
        joy_dict["EE_Y"] = (joy_dict["EE_Y_INC"] - joy_dict["EE_Y_DEC"])
        joy_dict["WAIST"] = (joy_dict["WAIST_CCW"] - joy_dict["WAIST_CW"])
        joy_dict["SPEED_TYPE"] = (joy_dict["SPEED_COARSE"] - joy_dict["SPEED_FINE"])
        joy_dict["SPEED"] = (joy_dict["SPEED_INC"] - joy_dict["SPEED_DEC"])
        # Enforce a safety limit on speed
        for key, value in joy_dict.items():
            joy_dict[key] = np.clip(value, -output_max, output_max)
    except IndexError as e:
        logger.error(f"IndexError: {e}")
        logger.error(f"Joy message had {len(axes)} axes and {len(buttons)} buttons.")
    
    return joy_dict