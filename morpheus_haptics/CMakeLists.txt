cmake_minimum_required(VERSION 3.8)
project(morpheus_haptics)

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
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(collision_serial src/collision_serial.cpp)
ament_target_dependencies(collision_serial rclcpp moveit_msgs sensor_msgs)

add_executable(collision_simple src/collision_simple.cpp)
ament_target_dependencies(collision_simple rclcpp moveit_msgs sensor_msgs geometry_msgs)

add_executable(collision_haptics src/collision_haptics.cpp)
ament_target_dependencies(collision_haptics rclcpp moveit_msgs sensor_msgs)

ament_package()
