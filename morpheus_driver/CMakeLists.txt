cmake_minimum_required(VERSION 3.8)
project(morpheus_driver)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(robotiq_driver REQUIRED)
find_package(onrobot_driver REQUIRED)

include_directories(include)

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
install(DIRECTORY config launch urdf
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
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)
ament_export_dependencies(control_msgs)
ament_export_dependencies(ur_robot_driver)
ament_export_dependencies(robotiq_driver)
ament_export_dependencies(onrobot_driver)
ament_export_include_directories(include)
ament_export_libraries(${LIBS})

ament_package()
