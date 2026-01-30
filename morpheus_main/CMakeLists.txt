cmake_minimum_required(VERSION 3.5)
project(morpheus_main)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(joy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)
  
# ------------------------------------------------------------------------------
# Launch Files
# ------------------------------------------------------------------------------
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.launch.xml" PATTERN "*.launch.yaml"
)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_dependencies(ament_cmake)
ament_export_dependencies(joy)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclpy)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
