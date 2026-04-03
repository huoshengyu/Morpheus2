

#! /usr/bin/env python3

# General Imports
import numpy as np
from copy import deepcopy
# ROS Imports
from rclpy.logging import get_logger

### Button mappings adapted from xsarm_joy.cpp from the Trossen Robotics Interbotix Library ###
# PS3 Controller button mappings
# Left stick:   x = -axes[0], y = axes[1]
# Right stick:  x = -axes[3], y = axes[4]
# Triggers:     LT = axes[2], RT = axes[5]
# Buttons:      [X, O, S, T, LB, RB, LT, RT, Share, Menu, Xbox, Lstick, Rstick, DU, DD, DL, DR]
ps3 = {
    "GRIPPER_PWM_DEC": 0,    # buttons start here
    "GRIPPER_OPEN": 1,
    "GRIPPER_PWM_INC": 2,
    "GRIPPER_CLOSE": 3,
    "EE_Y_INC": 4,
    "EE_Y_DEC": 5,
    "WAIST_CCW": 6,
    "WAIST_CW": 7,
    "SLEEP_POSE": 8,
    "HOME_POSE": 9,
    "TORQUE_ENABLE": 10,
    "FLIP_EE_X": 11,
    "FLIP_EE_ROLL": 12,
    "SPEED_INC": 13,
    "SPEED_DEC": 14,
    "SPEED_COARSE": 15,
    "SPEED_FINE": 16,
    "EE_X": 0,              # axes start here
    "EE_Z": 1,
    "EE_ROLL": 3,
    "EE_PITCH": 4,
    }

# PS4 Controller button mappings
# Left stick:   x = -axes[0], y = axes[1]
# Right stick:  x = -axes[3], y = axes[4]
# Triggers:     LT = axes[2], RT = axes[5]
# Dpad:         L/R = -axes[6], U/D = axes[7]
# Buttons:      [X, O, S, T, LB, RB, LT, RT, Share, Menu, Xbox, Lstick, Rstick]
ps4 = {
    "GRIPPER_PWM_DEC": 0,    # buttons start here
    "GRIPPER_OPEN": 1,
    "GRIPPER_PWM_INC": 2,
    "GRIPPER_CLOSE": 3,
    "EE_Y_INC": 4,
    "EE_Y_DEC": 5,
    "WAIST_CCW": 6,
    "WAIST_CW": 7,
    "SLEEP_POSE": 8,
    "HOME_POSE": 9,
    "TORQUE_ENABLE": 10,
    "FLIP_EE_X": 11,
    "FLIP_EE_ROLL": 12,
    "EE_X": 0,              # axes start here
    "EE_Z": 1,
    "EE_ROLL": 3,
    "EE_PITCH": 4,
    "SPEED_TYPE": 6,
    "SPEED": 7,
    }

# Xbox 360 Controller button mappings
# Left stick:   x = -axes[0], y = axes[1]
# Right stick:  x = -axes[3], y = axes[4]
# Triggers:     LT = axes[2], RT = axes[5]
# Dpad:         L/R = -axes[6], U/D = axes[7]
# Buttons:      [A, B, X, Y, LB, RB, Share, Menu, Xbox, Lstick, Rstick]
xbox360 = {
"GRIPPER_PWM_DEC": 0, # buttons start here
"GRIPPER_OPEN": 1,
"GRIPPER_CLOSE": 2,
"GRIPPER_PWM_INC": 3,
"WAIST_CCW": 4,
"WAIST_CW": 5,
"SLEEP_POSE": 6,
"HOME_POSE": 7,
"TORQUE_ENABLE": 8,
"FLIP_EE_X": 9,
"FLIP_EE_ROLL": 10,
"EE_X": 0,            # axes start here
"EE_Z": 1,
"EE_Y_INC": 2,
"EE_ROLL": 3,
"EE_PITCH": 4,
"EE_Y_DEC": 5,
"SPEED_TYPE": 6,
"SPEED": 7,
}

# Map of button mappings
button_mappings = {"ps3": ps3, "ps4": ps4, "xbox360": xbox360}

def get_joy_msg(joy_msg, joy_msg_mutex):
    # Retrieve joy_msg and return a safe copy
    joy_msg_mutex.acquire()
    msg = deepcopy(joy_msg)
    joy_msg_mutex.release()
    return msg

def joy_msg_to_dict(msg, controller_type="ps4", linear_scale=0.05, angular_scale=0.05, input_min=0.05, input_max=1, output_max=1.0, logger=get_logger("joy_utils")):
    # Get button mapping based on controller type
    if (controller_type == "xbox360"):
        button_mapping = xbox360
    elif (controller_type == "ps3"):
        button_mapping = ps3
    else:
        button_mapping = ps4
    
    # Convert joy_msg inputs to command outputs in 6 DOF
    axes = list(msg.axes)
    buttons = list(msg.buttons)

    # Set a deadzone and a limit in input values
    for i in range(len(axes)):
        if abs(axes[i]) < input_min:
            axes[i] = 0
        if abs(axes[i]) > input_max:
            axes[i] = axes[i] / abs(axes[i])

    # Retrieve movement controls by name
    joy_dict = {"EE_X": axes[button_mapping["EE_X"]] * linear_scale,
                    "EE_Z": axes[button_mapping["EE_Z"]] * linear_scale,  
                    "EE_PITCH": axes[button_mapping["EE_PITCH"]] * angular_scale, 
                    "EE_ROLL": axes[button_mapping["EE_ROLL"]] * angular_scale, 
                    "WAIST": (buttons[button_mapping["WAIST_CCW"]] - buttons[button_mapping["WAIST_CW"]]) * angular_scale,}
    if controller_type == "xbox360":
        joy_dict["EE_Y"] = (axes[button_mapping["EE_Y_INC"]] - axes[button_mapping["EE_Y_DEC"]]) * linear_scale
    else:
        joy_dict["EE_Y"] = (buttons[button_mapping["EE_Y_INC"]] - buttons[button_mapping["EE_Y_DEC"]]) * linear_scale

    # Enforce a safety limit on speed
    for key, value in joy_dict.items():
        joy_dict[key] = np.clip(value, -output_max, output_max)

    # Retrieve other controls by name
    try:
        joy_dict["GRIPPER_PWM_INC"] = buttons[button_mapping["GRIPPER_PWM_INC"]]
        joy_dict["GRIPPER_PWM_DEC"] = buttons[button_mapping["GRIPPER_PWM_DEC"]]
        joy_dict["GRIPPER_OPEN"] = buttons[button_mapping["GRIPPER_OPEN"]]
        joy_dict["GRIPPER_CLOSE"] = buttons[button_mapping["GRIPPER_CLOSE"]]
        joy_dict["SLEEP_POSE"] = buttons[button_mapping["SLEEP_POSE"]]
        joy_dict["HOME_POSE"] = buttons[button_mapping["HOME_POSE"]]
        joy_dict["TORQUE_ENABLE"] = buttons[button_mapping["TORQUE_ENABLE"]]
        joy_dict["FLIP_EE_X"] = buttons[button_mapping["FLIP_EE_X"]]
        joy_dict["FLIP_EE_ROLL"] = buttons[button_mapping["FLIP_EE_ROLL"]]
        if controller_type == "ps3":
            joy_dict["SPEED_TYPE"] = buttons[button_mapping["SPEED_COARSE"]] - buttons[button_mapping["SPEED_FINE"]]
            joy_dict["SPEED"] = buttons[button_mapping["SPEED_INC"]] - buttons[button_mapping["SPEED_DEC"]]
        else:
            joy_dict["SPEED_TYPE"] = axes[button_mapping["SPEED_TYPE"]]
            joy_dict["SPEED"] = axes[button_mapping["SPEED"]]
    except IndexError as e:
        logger.error(f"IndexError: {e}")
        logger.error("Check that controller type is set correctly where teleop_main.launch is called.")
    
    return joy_dict