cmake_minimum_required(VERSION 3.5)
project(morpheus_msgs)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

## Declare message files
set(msg_files
  "msg/ContactMap.msg"
  "msg/Float64Vector.msg"
  "msg/StringPair.msg"
  "msg/ArmJoy.msg"
)

## Declare service files
set(srv_files
  "srv/SpawnerMsgService.srv"
  "srv/SpawnerPresetService.srv"
  "srv/AttacherMsgService.srv"
  "srv/AttacherIndexService.srv"
)

## Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
