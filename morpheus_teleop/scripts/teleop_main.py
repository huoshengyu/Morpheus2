#! /usr/bin/env python3

# General Packages
import numpy as np
import quaternion
# ROS Packages
import rospy
import tf2_ros
import moveit_commander
import actionlib
# ROS Messages
import geometry_msgs.msg   
import sensor_msgs.msg 
import moveit_msgs.msg
import control_msgs.msg
# Individual Imports
from threading import Lock
from copy import deepcopy
from collections import deque
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_common_modules import angle_manipulation as ang
from scipy.spatial.transform import Rotation as R
# Local Imports
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand
from utils import switch_controller, list_controllers, command_robotiq2F85, command_onrobotRG2FT

### Button mappings from xsarm_joy.cpp
# PS3 Controller button mappings
ps3 = {"GRIPPER_PWM_DEC": 0,    # buttons start here
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
        "EE_PITCH": 4}

# PS4 Controller button mappings
ps4 = {"GRIPPER_PWM_DEC": 0,    # buttons start here
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
        "SPEED": 7}

# Xbox 360 Controller button mappings
xbox360 = {"GRIPPER_PWM_DEC": 0, # buttons start here
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
        "SPEED": 7}

from teleop_base import TeleopBase

class TeleopTwist(TeleopBase):
    def __init__(self):
        super().__init__()

        # Set controller type
        self.controller_type = rospy.get_param("~controller_type", "ps4")
        if (self.controller_type == "xbox360"):
            self.button_mapping = xbox360
        elif (self.controller_type == "ps3"):
            self.button_mapping = ps3
        else:
            self.button_mapping = ps4
        
        # Initialize frame swap variables
        self.reference_frame = "base"
        self.already_swapped = False

        # Set robot model
        self.robot_model = rospy.get_param("~robot_model", "vx300s")
        if self.robot_model == "vx300s":
            robot_name = rospy.get_namespace().strip("/")
            self.robot = InterbotixManipulatorXS(self.robot_model, robot_name=robot_name, moving_time=0.2, accel_time=0.1, init_node=False)

        # Set gripper type
        self.gripper_type = rospy.get_param("~gripper_type", "vx300s")

        # Listen for robot state
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get twist topic
        self.twist_topic = rospy.get_param("~twist_topic", "twist_controller/command")

        # Initialize twist command publishers and joystick subscriber
        self.twist_pub = rospy.Publisher(self.twist_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.gripper_pub_robotiq = rospy.Publisher('robotiq_2f_85_gripper/control', Robotiq2FGripper_robot_output, queue_size=1)
        self.gripper_pub_onrobot = rospy.Publisher('onrobot_rg2ft/command', RG2FTCommand, queue_size=1)
        self.gripper_pub_gazebo = actionlib.SimpleActionClient('gripper_action_controller/gripper_cmd', control_msgs.msg.GripperCommandAction)
        self.joy_sub = rospy.Subscriber("joy", sensor_msgs.msg.Joy, self.joy_callback)

        # Initialize variables for holding joystick inputs
        self.joy_msg = None
        self.joy_msg_mutex = Lock()
        self.input_dict = {}

        # Set limits on raw inputs and outputs
        self.input_min = 0.15
        self.input_max = 1.0
        self.output_max = 0.5

        # Set scaling factor on inputs
        self.linear_scale = 0.05
        self.angular_scale = 0.05

        # Initialize buffer arrays to enable moving average filtering of outputs
        self.twist_filter_size = 5  # Adjust this value for the moving average window size
        self.linear_x_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_y_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_z_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_x_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_y_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_z_buffer = deque(maxlen=self.twist_filter_size)
        self.buffer_list = [self.linear_x_buffer, self.linear_y_buffer, self.linear_z_buffer, self.angular_x_buffer, self.angular_y_buffer, self.angular_z_buffer]

        # Initialize moveit commander for giving motion plans to the robot, such as returning to home position
        self.robot_commander = moveit_commander.RobotCommander()
        self.planning_scene_interface = moveit_commander.PlanningSceneInterface()
        self.group_name = rospy.get_param("~arm_group", "arm")
        self.move_group_commander = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # Initialize joint commands for returning to home position
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.home = [0, -1.5708, 1.5708, -1.5708, -1.5708, 3.14]

    def get_joy_msg(self):
        # Retrieve joy_msg and return a safe copy
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()
        return msg

    def msg_to_dict(self, msg, linear_scale=None, angular_scale=None):
        # Convert joy_msg inputs to command outputs in 6 DOF
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        # Set a deadzone and a limit in input values
        for i in range(len(axes)):
            if abs(axes[i]) < self.input_min:
                axes[i] = 0
            if abs(axes[i]) > self.input_max:
                axes[i] = axes[i] / abs(axes[i])

        if linear_scale == None:
            linear_scale = self.linear_scale
        if angular_scale == None:
            angular_scale = self.angular_scale

        # Retrieve movement controls by name
        input_dict = {"EE_X": axes[self.button_mapping["EE_X"]] * linear_scale,
                        "EE_Z": axes[self.button_mapping["EE_Z"]] * linear_scale,  
                        "EE_PITCH": axes[self.button_mapping["EE_PITCH"]] * angular_scale, 
                        "EE_ROLL": axes[self.button_mapping["EE_ROLL"]] * angular_scale, 
                        "WAIST": (buttons[self.button_mapping["WAIST_CCW"]] - buttons[self.button_mapping["WAIST_CW"]]) * angular_scale,}
        if self.controller_type == "xbox360":
            input_dict["EE_Y"] = (axes[self.button_mapping["EE_Y_INC"]] - axes[self.button_mapping["EE_Y_DEC"]]) * linear_scale
        else:
            input_dict["EE_Y"] = (buttons[self.button_mapping["EE_Y_INC"]] - buttons[self.button_mapping["EE_Y_DEC"]]) * linear_scale

        # Enforce a safety limit on speed
        for key, value in input_dict.items():
            input_dict[key] = np.clip(value, -self.output_max, self.output_max)

        # Retrieve other controls by name
        try:
            input_dict["GRIPPER_PWM_INC"] = buttons[self.button_mapping["GRIPPER_PWM_INC"]]
            input_dict["GRIPPER_PWM_DEC"] = buttons[self.button_mapping["GRIPPER_PWM_DEC"]]
            input_dict["GRIPPER_OPEN"] = buttons[self.button_mapping["GRIPPER_OPEN"]]
            input_dict["GRIPPER_CLOSE"] = buttons[self.button_mapping["GRIPPER_CLOSE"]]
            input_dict["SLEEP_POSE"] = buttons[self.button_mapping["SLEEP_POSE"]]
            input_dict["HOME_POSE"] = buttons[self.button_mapping["HOME_POSE"]]
            input_dict["TORQUE_ENABLE"] = buttons[self.button_mapping["TORQUE_ENABLE"]]
            input_dict["FLIP_EE_X"] = buttons[self.button_mapping["FLIP_EE_X"]]
            input_dict["FLIP_EE_ROLL"] = buttons[self.button_mapping["FLIP_EE_ROLL"]]
            if self.controller_type == "ps3":
                input_dict["SPEED_TYPE"] = buttons[self.button_mapping["SPEED_COARSE"]] - buttons[self.button_mapping["SPEED_FINE"]]
                input_dict["SPEED"] = buttons[self.button_mapping["SPEED_INC"]] - buttons[self.button_mapping["SPEED_DEC"]]
            else:
                input_dict["SPEED_TYPE"] = axes[self.button_mapping["SPEED_TYPE"]]
                input_dict["SPEED"] = axes[self.button_mapping["SPEED"]]
        except IndexError as e:
            rospy.logerr(f"IndexError: {e}")
            rospy.logerr("Check that controller type is set correctly where teleop_main.launch is called.")
        
        return input_dict

    def rearrange_axes(self, command):
        # Command is the 6 DOF motion control inputs for the robot
        rearranged_command = command
        # Rearrange the control axes to make the controls more intuitive
        try:
            r_control_linear = np.array([[ 1,  0,  0],
                                            [ 0,  1,  0],
                                            [ 0,  0,  1]])
            r_control_angular = np.array([[ 1,  0,  0],
                                            [ 0,  1,  0],
                                            [ 0,  0,  1]])

            rearranged_command = np.concatenate((np.matmul(r_control_linear, command[:3]), np.matmul(r_control_angular, command[3:])))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Control axis rearrangement failed!")
        return rearranged_command
    
    def rotate_axes(self, command, target_frame = "world", source_frame = "tcp_link"):
        # Perform coordinate transfer by rotating to target_frame from source_frame
        rotated_axes = np.array(command)

        # Rotate the axes of the end effector to be more intuitive
        effector_offset = R.from_matrix([[ 0,  1,  0],
                                         [ 0,  0,  1],
                                         [-1,  0,  0]])
        
        # Lookup the transform (target_frame) from (source_frame)
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        
        transform_quaternion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

        transform_rotation = R.from_quat(transform_quaternion)

        # Apply the rotation matrix
        offset_axes = np.concatenate((effector_offset.apply(command[:3]), effector_offset.apply(command[3:])))
        rotated_command = np.concatenate((transform_rotation.apply(offset_axes[:3]), transform_rotation.apply(offset_axes[3:])))
        rotated_command = np.multiply(rotated_axes, [-1, -1, 1, -1, -1, 1])
        return rotated_command

    def publish(self, input_dict):
        # Publish pre-processed commands to the twist topic
        command = [input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"]]

        # If menu button is pressed, move to home
        if input_dict["HOME_POSE"] == 1:
            self.moveto(joint_pos=self.home)
            return

        # Append inputs on each axis to the respective buffers
        buffer_zip = zip(self.buffer_list, command)
        [buffer.append(axis) for buffer, axis in buffer_zip]

        # Obtain a moving average from each buffer and assign it to a new twist command
        twist = geometry_msgs.msg.Twist()
        twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z = [np.mean(np.array(buffer)) for buffer in self.buffer_list]
        self.twist_pub.publish(twist)

        # Convert Xbox button press into open/close command
        # Inputs: 
        #   name            type                limits (Robotiq 2F-85)          limits(OnRobot RG2FT)
        #   position        float [0,50]        [0,50] mm                       [0,100] mm
        #   force           float [0,40]        [20,235] N, capped to 40        [0,40] N
        #   speed           float [0,150]       [20,150] mm/s                   (No speed control, ranges ~[180, 25] mm/s as position increases)
        position = None
        if input_dict["GRIPPER_CLOSE"] == 1:
            position = 0 # Closed position, proportion
        elif input_dict["GRIPPER_OPEN"] == 1:
            position = 1 # Open position, proportion
        force = 20 # N
        speed = 100 # mm/s

        if position is not None:
            # Command Robotiq 2F-85 gripper
            if self.gripper_type == "robotiq":
                command_robotiq2F85(publisher=self.gripper_pub_robotiq, position=position, force=force, speed=speed)
            # Command OnRobot RG2FT gripper
            if self.gripper_type == "onrobot":
                command_onrobotRG2FT(publisher=self.gripper_pub_onrobot, position=position, force=force)
            # Command either UR5e gripper type in Gazebo via action server
            if self.gripper_type == "gazebo":
                gripper_command = control_msgs.msg.GripperCommandGoal()
                gripper_command.command.position = position
                gripper_command.command.max_effort = force
                self.gripper_pub_gazebo.send_goal(gripper_command)
        
        # Command Trossen robot
        if self.robot_model == "vx300s":
            robot_pose = self.robot.arm.get_ee_pose()
            pose_target = [pos + np.mean(np.array(buffer)) for pos, buffer in zip(robot_pose, self.buffer_list)]
            self.robot.arm.set_ee_pose_components(x=pose_target[0], y=pose_target[1], z=pose_target[2], roll=pose_target[3], pitch=pose_target[4], custom_guess=self.robot.arm.get_joint_commands(), blocking=False)
            if self.gripper_type == "vx300s" and position is not None:
                self.robot.gripper.gripper_controller(self.robot.gripper.gripper_value*(position * 2 - 1), 0.05)


    def update(self, msg):
        if msg is not None:
            input_dict = self.msg_to_dict(msg)
            input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"] = self.rearrange_axes([input_dict["EE_X"], input_dict["EE_Y"], input_dict["EE_Z"], input_dict["EE_ROLL"], input_dict["EE_PITCH"], input_dict["WAIST"]])
            self.input_dict = input_dict
            return input_dict
        else:
            return None

    def loop_once(self):
        if self.joy_msg is not None:
            input_dict = self.update(self.joy_msg)
            if input_dict is not None:
                self.publish(input_dict)

    def joy_callback(self, msg):
        # Retrieve joy_msg
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()
        # Process and publish
        self.loop_once()

    # Get the robot's current state
    def get_joint_state(self):
        return self.move_group_commander.get_current_joint_values()
    
    # Calculate the yaw/waist rotation of the robot
    def get_yaw(self):
        robot_state = self.get_joint_state()
        return robot_state[0]
    
    # Get the robot's current end effector pose
    def get_ee_pose(self):
        return self.move_group_commander.get_current_pose()
    
    # Calculate the transform of the target frame w.r.t. the source frame
    def get_T(self, target_frame, source_frame="world", source_time=rospy.Time(0), timeout=rospy.Duration(1.0)):
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, source_time, timeout)
        return transform

    # Calculate the transform of the end-effector w.r.t. the robot's shoulder/yaw position
    def get_T_yb(self):
        T_ee = self.get_T(self.move_group_commander.get_end_effector_link())
        yaw = self.get_yaw()
        T_yaw = ang.yawToRotationMatrix(yaw)
        T_ee_yaw = np.dot(ang.transInv(T_yaw), T_ee)
        return T_ee_yaw


    def moveto(self, joint_pos=None):
        # Command robot to move to specified joint position, such as home position
        if joint_pos is None:
            joint_pos = self.home
        # List expected twist and position controllers
        twist_controllers = ["twist_controller", "cartesian_compliance_controller"]
        pos_controllers = ["pos_joint_traj_controller"]

        # Retrieve list of active controllers
        controllers = list_controllers().controller
        running_controllers = [controller.name for controller in controllers if controller.state=="running"]
        running_twist_controllers = list(set(running_controllers) & set(twist_controllers))
        stopped_controllers = [controller.name for controller in controllers if (controller.state=="initialized" or controller.state=="stopped")]
        stopped_pos_controllers = list(set(stopped_controllers) & set(pos_controllers))

        try:
            # Switch to position controller
            print("Stopping controllers: " + str(running_twist_controllers))
            print("Starting controllers: " + str(stopped_pos_controllers))
            switch_controller(stop_controllers=running_twist_controllers, start_controllers=stopped_pos_controllers)
            print("Controller states: " + str([controller.state for controller in controllers]))

            # Execute a position command
            print("Executing move command")
            self.move_group_commander.go(joint_pos, wait=True)

            # Stop after completion
            print("Stopping after move command")
            self.move_group_commander.stop()

            # Reset target pose to match current pose


            # Switch back to twist controller
            print("Stopping controllers: " + str(stopped_pos_controllers))
            print("Starting controllers: " + str(running_twist_controllers))
            switch_controller(stop_controllers=stopped_pos_controllers, start_controllers=running_twist_controllers)
            print("Controller states: " + str([controller.state for controller in controllers]))
            return True
        except (...) as e:
            print("Twist controller moveto() failed: %s"%e)
            # If failed, attempt to restore status quo
            print("Attempting to restore normal state")
            print("Stopping move command")
            self.move_group_commander.stop()
            print("Stopping controllers: " + str(stopped_pos_controllers))
            print("Starting controllers: " + str(running_twist_controllers))
            switch_controller(stop_controllers=stopped_pos_controllers, start_controllers=running_twist_controllers, strictness=1)
            print("Controller states: " + str([controller.state for controller in controllers]))
            return False

if __name__ == '__main__':
    rospy.init_node('teleop_twist')

    teleop_twist = TeleopTwist()
    
    rospy.spin()
