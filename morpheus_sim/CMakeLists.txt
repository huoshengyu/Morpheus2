cmake_minimum_required(VERSION 3.8)
project(morpheus_sim)

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
find_package(rclpy REQUIRED)

include_directories(include)
  
# ------------------------------------------------------------------------------
# Shared Resources
# ------------------------------------------------------------------------------
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
