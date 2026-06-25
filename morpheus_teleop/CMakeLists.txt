cmake_minimum_required(VERSION 3.8)
project(morpheus_teleop)

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
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_python
  rclpy
  rclcpp
  joy
  std_msgs
  sensor_msgs
  geometry_msgs
  morpheus_msgs
)

# ------------------------------------------------------------------------------
# Libraries and Headers
# ------------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

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
file(GLOB PYTHON_SCRIPTS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${PROJECT_NAME}/*.py
)
install(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# ------------------------------------------------------------------------------
# Shared Resources
# ------------------------------------------------------------------------------
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(${LIBS})

ament_package()
