cmake_minimum_required(VERSION 3.5)
project(morpheus_sim)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)
  
# ------------------------------------------------------------------------------
# Shared Resources
# ------------------------------------------------------------------------------
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
