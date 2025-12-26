# CompliantTrajectoryControl

The CompliantTrajectoryControl ROS package simplifies joint trajectory planning in MoveIt while ensuring compliance and adhering to cartesian path and goal constraints. It offers an action server that accepts FollowCompliantTrajectory actions and manages the robot arm with compliance in Cartesian space.

## Dependencies
Before utilizing this package, ensure that you have the following dependencies installed:

1. [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)
2. [cartesian_control_msgs](https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs)

## Overview
This package functions as a middleware layer, converting a joint trajectory into a Cartesian trajectory. It subsequently interpolates the cartesian trajectory and publishes target frames and wrench to a Cartesian compliance controller while validating path and goal tolerances.

## Configuration
For configuration, refer to the following YAML file:

```yaml
cartesian_compliance_controller:
  type: "velocity_controllers/CartesianComplianceController"
  end_effector_link: "tcp_link"
  robot_base_link: "base_link"
  ft_sensor_ref_link: "wrist_3_link"
  compliance_ref_link: "tcp_link"
  target_frame_topic: "target_frame"
  joints: &robot_joints
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
  # Further configure your compliance controller (gains, stiffness, etc)

# Ensure this name matches the corresponding node in the launch file
compliant_traj_action_server:
  end_effector_link: "tcp_link"
  robot_base_link: "base_link"
  joints: *robot_joints
  path_tolerance:
    pose_error: {trans_x: 0.3, trans_y: 0.3, trans_z: 0.3, rot_x: 1.0, rot_y: 1.0, rot_z: 1.0}
  goal_tolerance:
    pose_error: {trans_x: 0.2, trans_y: 0.2, trans_z: 0.2, rot_x: 0.5, rot_y: 0.5, rot_z: 0.5}
  goal_time_tolerance: 3.0
```

## Usage

Follow this example to call the action server:

```python
# Set up move group commander
move_group = moveit_commander.MoveGroupCommander("your_move_group_name")

# Generate a plan to a named target
move_group.set_start_state_to_current_state()
self.move_group.set_named_target("your_move_group_target")
success, robot_traj, time, error_code = self.move_group.plan()
joint_trajectory = robot_traj.joint_trajectory

# Create an action client for the FollowCompliantTrajectoryAction
client = actionlib.SimpleActionClient('/compliant_traj_action_server/follow_compliant_trajectory', FollowCompliantTrajectoryAction)
client.wait_for_server()

# Create a FollowCompliantTrajectoryGoal
goal = FollowCompliantTrajectoryGoal()
goal.joint_trajectory = joint_trajectory
goal.wrench.header.frame_id = "base_link"
goal.wrench.wrench.force.y = 10.0

# Send the goal to the action server
client.send_goal(goal)

# Wait for the result
client.wait_for_result()
```
