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

add_subdirectory(ros_utils)

add_subdirectory(../../cpp ${CMAKE_CURRENT_BINARY_DIR}/rko_lio_cpp)

add_library(rko_lio.ros STATIC)
add_library(rko_lio::ros ALIAS rko_lio.ros)
target_sources(
  rko_lio.ros
  PRIVATE node.cpp
  PUBLIC FILE_SET
         HEADERS
         FILES
         ../rko_lio/node.hpp
         BASE_DIRS
         ..)
ament_target_dependencies(
  rko_lio.ros
  PUBLIC
  rclcpp
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs)
target_link_libraries(rko_lio.ros PUBLIC rko_lio::ros_utils rko_lio::core
                                         Sophus::Sophus Eigen3::Eigen)

add_executable(offline_node offline_node.cpp)
target_link_libraries(offline_node PRIVATE rko_lio::ros)

add_library(online_node_component SHARED online_node.cpp)
ament_target_dependencies(online_node_component PUBLIC rclcpp_components)
target_link_libraries(online_node_component PRIVATE rko_lio::ros)
