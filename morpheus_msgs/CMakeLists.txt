cmake_minimum_required(VERSION 3.8)
project(morpheus_msgs)

# Ensure the package correctly uses C++17 and C99 standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function declarations
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-redundant-decls")
endif()

## Find libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ------------------------------------------------------------------------------
# Interfaces
# ------------------------------------------------------------------------------
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
  "srv/TwistConverter.srv"
)

## Declare action files
set(action_files
  "action/MoveToNamedTarget.action"
)

## Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
