cmake_minimum_required(VERSION 3.0.2)
project(compliant_trajectory_control)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    message_generation
    std_msgs
    geometry_msgs
    trajectory_msgs
    cartesian_control_msgs
    dynamic_reconfigure
    actionlib_msgs
)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Tolerance6DOF.cfg
  cfg/GoalTimeTolerance.cfg
)

add_action_files(
  FILES
  FollowCompliantTrajectory.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  cartesian_control_msgs
  dynamic_reconfigure
  actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    trajectory_msgs
    cartesian_control_msgs
    actionlib_msgs
)

catkin_install_python(
    PROGRAMS 
        scripts/follow_compliant_trajectory_action_server
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
