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
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
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

# ------------------------------------------------------------------------------
# Libraries and Headers
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}
  SHARED
  include/morpheus_collision/collision_detector_allocator_distance_field.hpp
  include/morpheus_collision/collision_detector_allocator_hybrid.hpp
  src/collision.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} PUBLIC 
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

install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ------------------------------------------------------------------------------
# C++ Executables
# ------------------------------------------------------------------------------
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
target_link_libraries(collision 
  ${PROJECT_NAME}
)

install(TARGETS collision
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Shared Resources
# ------------------------------------------------------------------------------
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
