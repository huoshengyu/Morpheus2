cmake_minimum_required(VERSION 3.5)
project(morpheus_collision)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(morpheus_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME} src/collision.cpp)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometric_shapes
    morpheus_msgs
    geometry_msgs
    std_msgs
    moveit_msgs
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_core
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(morpheus_msgs)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(rclcpp)
ament_export_dependencies(moveit_core)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(moveit_visual_tools)
ament_export_dependencies(geometric_shapes)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
