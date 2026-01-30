cmake_minimum_required(VERSION 3.5)
project(morpheus_teleop)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(morpheus_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# ------------------------------------------------------------------------------
# C++ Executables
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_xsarm_joy src/xsarm_joy.cpp)
ament_target_dependencies(${PROJECT_NAME}_xsarm_joy rclcpp sensor_msgs morpheus_msgs)

# Install C++ executables
install(TARGETS
  ${PROJECT_NAME}_xsarm_joy
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Python Modules & Scripts
# ------------------------------------------------------------------------------
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS  # Keep original executable permissions
  FILES_MATCHING PATTERN "*.py"
)

# ------------------------------------------------------------------------------
# Launch Files
# ------------------------------------------------------------------------------
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.launch.xml" PATTERN "*.launch.yaml"
)

# ------------------------------------------------------------------------------
# Interfaces
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME} "srv/TwistConverter.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
  std_msgs)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_dependencies(ament_cmake)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rclpy)
ament_export_dependencies(rclcpp)
ament_export_dependencies(joy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(morpheus_msgs)
ament_export_include_directories(include)
ament_export_libraries(${LIBS})

ament_package()
