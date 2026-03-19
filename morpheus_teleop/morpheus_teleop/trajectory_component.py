#! /usr/bin/env python3

"""Simple node for planning and executing trajectories using MoveIt"""

# ROS Imports
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
# ROS Message Imports
from std_msgs.msg import String
# MoveIt2 Imports
from moveit.planning import MoveItPy
# Local Imports
from wait_for_topic import wait_for_topic

class TrajectoryComponent(Node):
    def __init__(self, node_name="trajectory_component_node", moveit=None, planning_group="arm", wait_topic="robot_description_semantic"):
        super().__init__(node_name)
        
        # Store arguments
        self.planning_group = planning_group
        self.wait_topic = wait_topic

        # Initialize moveit for giving motion plans to the robot, such as returning to home position
        self.initialized = False
        if not moveit:
            moveit = self.initialize_moveitpy()
        self.moveit = moveit
        self.initialized = True
        
        # Get moveit components needed for planning and executing trajectories
        self.planning_group = planning_group
        self.planning_component = self.moveit.get_planning_component(self.planning_group)
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
    
    def initialize_moveitpy(self):
        # Wait for robot description semantic to be available
        wait_for_message(String, self, self.wait_topic)
        # Create moveitpy instance
        moveit = MoveItPy(node_name="moveit_py_trajectory_component")
        return moveit
        
    def plan_and_execute(self,
                configuration_name=None, 
                robot_state=None, 
                pose_stamped_msg=None, pose_link=None, 
                motion_plan_constraints=None,
                single_plan_parameters=None,
                multi_plan_parameters=None,):
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
        execute_result = self.execute(plan_result)
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
        self.get_logger().info("Setting MoveIt goal state...")
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
        self.get_logger().info("Planning trajectory...")
        plan_result = self.planning_component.plan(single_plan_parameters=single_plan_parameters,
                                                   multi_plan_parameters=multi_plan_parameters,)
        if plan_result:
                self.get_logger().info("Planning successful")
        else:
                self.get_logger().error("Planning failed")
        return plan_result
    
    def execute(self, plan_result):
        """
        Execute a given trajectory plan.
        The active ROS controller must have an action server that accepts control_msgs/FollowJointTrajectoryAction for this to work.
        Such a ROS controller should show an action named "follow_joint_trajectory" when running "ros2 action list".
        The correct action type is control_msgs/action/FollowJointTrajectory.
        """
        # Check plan result
        if not plan_result:
            self.get_logger().error("Planning failed")
            return plan_result
        # Execute trajectory
        else:
            self.get_logger().info("Executing trajectory...")
            trajectory = plan_result.trajectory
            execute_result = self.moveit.execute(trajectory, controllers=[])
            return execute_result

