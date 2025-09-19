/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "point_cloud_read.hpp"
// stl
#include <cstddef>
#include <functional>
#include <stdexcept>

namespace rko_lio::ros_utils {
using Vector3dVector = std::vector<Eigen::Vector3d>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

Vector3dVector point_cloud2_to_eigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  const size_t point_count = static_cast<size_t>(msg->height) * msg->width;
  Vector3dVector points;
  points.reserve(point_count);
  sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
  for (size_t i = 0; i < point_count; ++i, ++msg_x, ++msg_y, ++msg_z) {
    points.emplace_back(*msg_x, *msg_y, *msg_z);
  }
  return points;
}

std::tuple<Vector3dVector, std::vector<double>>
point_cloud2_to_eigen_with_timestamps(const PointCloud2::ConstSharedPtr& msg) {
  using sensor_msgs::PointCloud2ConstIterator;
  // getting points and time in a single cycle loop
  const size_t point_count = static_cast<size_t>(msg->height) * msg->width;
  Vector3dVector points;
  points.reserve(point_count);
  PointCloud2ConstIterator<float> msg_x(*msg, "x");
  PointCloud2ConstIterator<float> msg_y(*msg, "y");
  PointCloud2ConstIterator<float> msg_z(*msg, "z");

  std::vector<double> raw_timestamps;
  raw_timestamps.reserve(point_count);

  const auto& timestamp_field = std::invoke([&msg]() -> PointField {
    for (const PointField& field : msg->fields) {
      if ((field.name == "t" || field.name == "timestamp" || field.name == "time" || field.name == "stamps")) {
        if (field.count != 0U) {
          return field;
        }
      }
    }
    throw std::invalid_argument("Point cloud needs timestamps for deskewing");
  });

  // templated lambda (auto) ftw
  auto extract_points_and_timestamps = [&](auto&& time_iter) {
    for (size_t i = 0; i < point_count; ++i, ++msg_x, ++msg_y, ++msg_z, ++time_iter) {
      points.emplace_back(*msg_x, *msg_y, *msg_z);
      raw_timestamps.emplace_back(static_cast<double>(*time_iter));
    }
  };

  switch (timestamp_field.datatype) {
  case PointField::UINT32: {
    PointCloud2ConstIterator<uint32_t> msg_time(*msg, timestamp_field.name);
    extract_points_and_timestamps(msg_time);
    break;
  }
  case PointField::FLOAT32: {
    PointCloud2ConstIterator<float> msg_time(*msg, timestamp_field.name);
    extract_points_and_timestamps(msg_time);
    break;
  }
  case PointField::FLOAT64: {
    PointCloud2ConstIterator<double> msg_time(*msg, timestamp_field.name);
    extract_points_and_timestamps(msg_time);
    break;
  }
  default:
    throw std::invalid_argument("Unsupported timestamp field datatype. Please open an issue.");
  }

  return {points, raw_timestamps};
}
} // namespace rko_lio::ros_utils
