cmake_minimum_required(VERSION 3.8)
project(morpheus_trajectory)

# Ensure the package correctly uses C++17 and C99 standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(morpheus_teleop REQUIRED)
find_package(morpheus_msgs REQUIRED)

add_library(move_to_named_target SHARED
  src/move_to_named_target_action_server.cpp)
target_include_directories(move_to_named_target PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(move_to_named_target
  PRIVATE "MORPHEUS_TRAJECTORY_BUILDING_DLL")
set_target_properties(move_to_named_target PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
rclcpp_components_register_node(move_to_named_target PLUGIN "morpheus_trajectory::MoveToNamedTargetActionServer" EXECUTABLE move_to_named_target_action_server)
ament_target_dependencies(move_to_named_target rclcpp rclcpp_action rclcpp_components controller_manager_msgs moveit_ros_planning_interface morpheus_msgs)

add_executable(visualization src/trajectory.cpp)
ament_target_dependencies(visualization rclcpp moveit_ros_planning_interface moveit_visual_tools morpheus_teleop)

install(TARGETS 
  move_to_named_target visualization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ------------------------------------------------------------------------------
# Shared Resources
# ------------------------------------------------------------------------------
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(std_msgs)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(controller_manager_msgs)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_visual_tools)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(ur_msgs)
ament_export_dependencies(morpheus_teleop)
ament_export_dependencies(morpheus_msgs)
ament_export_include_directories(include)

ament_package()
