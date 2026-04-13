cmake_minimum_required(VERSION 3.8)
project(morpheus_collision)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(morpheus_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)

include_directories(include)

add_executable(collision src/collision.cpp)
ament_target_dependencies(collision
  rclcpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_visual_tools
  moveit_msgs
  morpheus_msgs
  geometric_shapes
)

install(TARGETS 
  collision
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Shared Resources
# ------------------------------------------------------------------------------
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(collision)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(moveit_ros_move_group)
ament_export_dependencies(moveit_visual_tools)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(morpheus_msgs)
ament_export_dependencies(geometric_shapes)

ament_package()
