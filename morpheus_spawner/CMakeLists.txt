cmake_minimum_required(VERSION 3.8)
project(morpheus_spawner)

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
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(morpheus_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(include)

add_executable(spawner src/spawner.cpp)
ament_target_dependencies(spawner rclcpp std_msgs tf2_eigen moveit_ros_planning_interface moveit_ros_planning morpheus_msgs)

file(GLOB PYTHON_SCRIPTS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${PROJECT_NAME}/*.py
)

install(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(morpheus_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rclpy)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_eigen)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
