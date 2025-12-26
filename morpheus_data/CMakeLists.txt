cmake_minimum_required(VERSION 3.0.2)
project(morpheus_data)

# If any .cpp uses <filesystem>, use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  std_msgs
  geometry_msgs
  morpheus_msgs
  interbotix_xs_msgs
  interbotix_xsarm_control
  sensor_msgs
  tf
  rosbag
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES    # (none)
  # CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  # include
)

# ---- Executables ----
add_executable(trossen_csv     src/trossen_csv.cpp)
target_link_libraries(trossen_csv ${catkin_LIBRARIES})
add_dependencies(trossen_csv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(trossen_tf_csv  src/trossen_tf_csv.cpp)
target_link_libraries(trossen_tf_csv ${catkin_LIBRARIES})
add_dependencies(trossen_tf_csv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(Backup_Bag     src/Backup_Bag.cpp)
target_link_libraries(Backup_Bag ${catkin_LIBRARIES})
add_dependencies(Backup_Bag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# If you’re on GCC < 9 and using <filesystem>, uncomment:
# target_link_libraries(trossen_csv stdc++fs)
# target_link_libraries(trossen_tf_csv stdc++fs)

# ---- Python scripts (installed launcher for roslaunch) ----
# Ensure scripts/UCD_session.py exists, has a shebang (#!/usr/bin/env python or python3),
# and is chmod +x.
# catkin_install_python(
#   PROGRAMS
#     scripts/UCD_session.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ---- Install compiled nodes ----
install(TARGETS
  trossen_csv
  trossen_tf_csv
  Backup_Bag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- Install headers (only if you actually have some) ----
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
endif()
