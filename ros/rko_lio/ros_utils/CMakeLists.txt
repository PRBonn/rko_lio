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

add_library(rko_lio.ros_utils STATIC)
add_library(rko_lio::ros_utils ALIAS rko_lio.ros_utils)
compat_target_sources(
  rko_lio.ros_utils
  PRIVATE point_cloud_read.cpp point_cloud_write.cpp rosbag_utils.cpp
  PUBLIC
  FILE_SET
  HEADERS
  FILES ../../rko_lio/ros_utils/point_cloud_read.hpp
        ../../rko_lio/ros_utils/point_cloud_write.hpp
        ../../rko_lio/ros_utils/ros_utils.hpp
        ../../rko_lio/ros_utils/ros_vectors.hpp
        ../../rko_lio/ros_utils/rosbag_utils.hpp
        ../../rko_lio/ros_utils/time.hpp
        ../../rko_lio/ros_utils/transforms.hpp
  BASE_DIRS ../..)
target_link_libraries(
  rko_lio.ros_utils
  PUBLIC # ros deps
         ${sensor_msgs_TARGETS}
         rclcpp::rclcpp
         rosbag2_cpp::rosbag2_cpp
         rosbag2_storage::rosbag2_storage
         sensor_msgs::sensor_msgs_library
         tf2::tf2
         tf2_ros::static_transform_broadcaster_node
         tf2_ros::tf2_ros
         # ros deps end
         Sophus::Sophus
         Eigen3::Eigen)
# TODO: add the ros_utils as a library export target to enable reuse
