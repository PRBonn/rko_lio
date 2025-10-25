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

# library: rko_lio::ros::utils
add_subdirectory(utils)

# library: rko_lio::ros
add_library(rko_lio.ros STATIC)
add_library(rko_lio::ros ALIAS rko_lio.ros)

compat_target_sources(
  rko_lio.ros
  PRIVATE node.cpp
  PUBLIC
  FILE_SET
  HEADERS
  FILES ../../rko_lio/ros/node.hpp
  BASE_DIRS ../..)

target_link_libraries(
  rko_lio.ros
  PRIVATE nlohmann_json::nlohmann_json
  PUBLIC rko_lio::ros::utils
         # ros deps
         ${geometry_msgs_TARGETS}
         ${nav_msgs_TARGETS}
         ${sensor_msgs_TARGETS}
         ${std_msgs_TARGETS}
         rclcpp::rclcpp
         sensor_msgs::sensor_msgs_library
         tf2::tf2
         tf2_ros::static_transform_broadcaster_node
         tf2_ros::tf2_ros
         # ros deps end
         rko_lio::core
         Sophus::Sophus
         Eigen3::Eigen)

set_target_properties(rko_lio.ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

# executable: offline_node
add_executable(offline_node offline_node.cpp)
target_link_libraries(offline_node PRIVATE rko_lio::ros)
set_target_properties(offline_node PROPERTIES POSITION_INDEPENDENT_CODE ON)

# component: online_node_component
add_library(online_node_component SHARED online_node.cpp)
target_link_libraries(
  online_node_component
  PRIVATE rko_lio::ros
          # ros deps
          rclcpp_components::component rclcpp_components::component_manager)
