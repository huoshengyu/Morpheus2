# ---------------------------------------------------------------------------
# Morpheus2: ROS2 control and feedback for manual teleoperation research
# Copyright (C) 2026 Brian Huo
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.8)
project(morpheus_data)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ------------------------------------------------------------------------------
# Install Python Executables & Scripts
# ------------------------------------------------------------------------------
install(PROGRAMS
  scripts/session_manager.py
  scripts/UCDsession.py
  scripts/data_recorder.py
  scripts/trossen_tf_csv.py
  scripts/trossen_tf_csv
  scripts/backup_bag.py
  scripts/Backup_Bag
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install Launch & Config
# ------------------------------------------------------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)

ament_package()
