# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.28)
project(rko_lio LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependencies.cmake)

find_package(Eigen3 3.4 REQUIRED CONFIG)
find_package(Sophus REQUIRED CONFIG)
find_package(TBB REQUIRED CONFIG)
find_package(Bonxai REQUIRED CONFIG)
find_package(nlohmann_json REQUIRED CONFIG)

add_subdirectory(cpp/rko_lio/core)

option(RKO_LIO_BUILD_ROS "Build the ROS wrapper for RKO LIO" ON)
find_package(ament_cmake QUIET)

if(RKO_LIO_BUILD_ROS AND ament_cmake_FOUND)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosbag2_cpp REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  find_package(rclcpp_components REQUIRED)

  add_subdirectory(ros/rko_lio)

  # ament macros cannot be in a CMake subdirectory, even the rclcpp stuff. such
  # a pain to debug
  rclcpp_components_register_node(
    online_node_component
    PLUGIN
    "rko_lio::ros::OnlineNode"
    EXECUTABLE
    online_node)

  install(
    TARGETS online_node_component offline_node
    EXPORT rko_lioTargets
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY ros/config ros/launch DESTINATION share/rko_lio)

  ament_export_targets(rko_lioTargets)

  ament_package()
else()
  message(STATUS "Building with cmake")
endif()
