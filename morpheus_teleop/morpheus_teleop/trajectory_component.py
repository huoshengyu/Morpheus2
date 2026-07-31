#! /usr/bin/env python3

"""Simple node for planning and executing trajectories using MoveIt"""

# General Imports
import numpy as np
# ROS Imports
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.wait_for_message import wait_for_message
from controller_manager import switch_controllers, list_controllers
# ROS Message Imports
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan
# MoveIt2 Imports


class TrajectoryComponent():
    def __init__(self, node, moveit=None, planning_group="arm", wait_topic="/robot_description_semantic"):
        # Store parent node
        self.node = node
        
        # Store arguments
        self.planning_group = planning_group
        self.wait_topic = wait_topic

        # Initialize moveit for giving motion plans to the robot, such as returning to home position
        self.plan_client = self.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.execute_client = ActionClient(self.node, "/scaled_joint_trajectory_controller/follow_joint_trajectory", FollowJointTrajectory,)
        self.initialized = False
        if not moveit:
            moveit = self.initialize_moveitpy()
        self.moveit = moveit
        self.initialized = True
        
        # Get moveit components needed for planning and executing trajectories
        self.planning_group = planning_group
        self.planning_component = self.moveit.get_planning_component(self.planning_group)
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.group_name = self.planning_group
        
        self.node.get_logger().info(f"Initialized teleop trajectory component")
    
    def initialize_moveitpy(self):
        # Wait for robot description semantic to be available
        self.node.get_logger().info(f"Waiting for topic {self.wait_topic}...")
        wait_for_message(String, self.node, self.wait_topic)
        self.node.get_logger().info(f"Got message on topic {self.wait_topic}")
        # Create moveitpy instance
        moveit = MoveItPy(node_name="moveit_py")
        return moveit
        
    def plan_and_execute(self,
                configuration_name=None, 
                robot_state=None, 
                pose_stamped_msg=None, pose_link=None, 
                motion_plan_constraints=None,
                single_plan_parameters=None,
                multi_plan_parameters=None,
                controller_manager_name="controller_manager", 
                deactivate_controllers="", 
                activate_controllers="scaled_joint_trajectory_controller", 
                strictness="strict",):
        """
        Plan and execute a trajectory to the given goal state.
        Arguments are the same as those of MoveIt's planning_component.set_goal_state() + planning_component.plan()
        """
        # Plan trajectory
        plan_result = self.plan(configuration_name=configuration_name, 
                                robot_state=robot_state, 
                                pose_stamped_msg=pose_stamped_msg, pose_link=pose_link, 
                                motion_plan_constraints=motion_plan_constraints,
                                single_plan_parameters=single_plan_parameters,
                                multi_plan_parameters=multi_plan_parameters)
        # Execute trajectory
        execute_result = self.execute(plan_result,
                                      controller_manager_name=controller_manager_name,
                                      deactivate_controllers=deactivate_controllers,
                                      activate_controllers=activate_controllers,
                                      strictness=strictness,)
        return execute_result
    
    def plan(self,
             configuration_name=None, 
             robot_state=None, 
             pose_stamped_msg=None, pose_link=None, 
             motion_plan_constraints=None,
             single_plan_parameters=None,
             multi_plan_parameters=None,):
        """
        Plan a trajectory to the given goal state.
        Arguments are the same as those of MoveIt's planning_component.set_goal_state() + planning_component.plan()
        """
        self.node.get_logger().info("Setting MoveIt goal state...")
        # Set start position to current position
        self.planning_component.set_start_state_to_current_state()
        # Set goal position
        self.planning_component.set_goal_state(configuration_name=configuration_name, 
                                               robot_state=robot_state, 
                                               pose_stamped_msg=pose_stamped_msg, pose_link=pose_link, 
                                               motion_plan_constraints=motion_plan_constraints,
                                               single_plan_parameters=single_plan_parameters,
                                               multi_plan_parameters=multi_plan_parameters)
        # Plan trajectory
        self.node.get_logger().info("Planning trajectory...")
        plan_result = self.planning_component.plan(single_plan_parameters=single_plan_parameters,
                                                   multi_plan_parameters=multi_plan_parameters,)
        if plan_result:
                self.node.get_logger().info("Planning successful")
        else:
                self.node.get_logger().error("Planning failed")
        return plan_result
    
    def execute(self,
                plan_result, 
                controller_manager_name="controller_manager", 
                deactivate_controllers="", 
                activate_controllers="scaled_joint_trajectory_controller", 
                strictness="strict",):
        """
        Execute a given trajectory plan.
        The active ROS controller must have an action server that accepts control_msgs/FollowJointTrajectoryAction for this to work.
        Such a ROS controller should show an action named "follow_joint_trajectory" when running "ros2 action list".
        The correct action type is control_msgs/action/FollowJointTrajectory.
        """
        # Check plan result
        if not plan_result:
            self.node.get_logger().error("Planning invalid, aborting execution")
            return plan_result
        # Swap to trajectory controller and execute trajectory
        else:
            # Identify current joint controller if not specified
            if not deactivate_controllers:
                controller_list = list_controllers()
                active_joint_controllers = [controller for controller in controller_list 
                                            if any("joint/" in interface for interface in controller.claimed_interfaces)]
                deactivate_controllers = active_joint_controllers
            # Swap controllers
            if deactivate_controllers != activate_controllers:
                self.node.get_logger().info(f"Switching from {deactivate_controllers} to {activate_controllers}...")
                switch_controllers(self.node, 
                                   controller_manager_name=controller_manager_name,
                                   deactivate_controllers=deactivate_controllers,
                                   activate_controllers=activate_controllers,
                                   strictness=strictness,)
            # Execute trajectory
            self.node.get_logger().info("Executing trajectory...")
            trajectory = plan_result.trajectory
            execute_result = self.moveit.execute(trajectory, controllers=[activate_controllers])
            self.moveit.wait_until_executed()
            # Swap controllers back
            if deactivate_controllers != activate_controllers:
                self.node.get_logger().info(f"Switching back from {activate_controllers} to {deactivate_controllers}...")
                switch_controllers(self.node, 
                                   controller_manager_name=controller_manager_name,
                                   deactivate_controllers=activate_controllers,
                                   activate_controllers=deactivate_controllers,
                                   strictness=strictness,)
            return execute_result

