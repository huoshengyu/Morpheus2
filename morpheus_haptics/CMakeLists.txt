cmake_minimum_required(VERSION 3.5)
project(morpheus_haptics)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(morpheus_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

add_executable(collision_serial src/collision_serial.cpp)
ament_target_dependencies(collision_serial rclcpp std_msgs moveit_msgs sensor_msgs moveit_ros_planning_interface)
add_executable(collision_simple src/collision_simple.cpp)
add_executable(collision_haptics src/collision_haptics.cpp)

set(LIBRARY_DIRS ${ament_cmake_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES} ${morpheus_msgs_LIBRARIES}
  ${moveit_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${moveit_core_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${rosidl_default_generators_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES} ${moveit_visual_tools_LIBRARIES})
target_link_libraries(collision_serial ${LIBRARY_DIRS})
target_link_libraries(collision_simple ${LIBRARY_DIRS})
target_link_libraries(collision_haptics ${LIBRARY_DIRS})
set(LIBS ${ament_cmake_LIBRARIES} ${moveit_ros_planning_interface_LIBRARIES}
  ${morpheus_msgs_LIBRARIES} ${moveit_msgs_LIBRARIES} ${rclcpp_LIBRARIES}
  ${moveit_core_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${moveit_ros_planning_LIBRARIES}
  ${moveit_visual_tools_LIBRARIES})

install(TARGETS collision_serial
  RUNTIME
  DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(morpheus_msgs)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(moveit_core)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_visual_tools)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
