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
#include "rko_lio/core/error.hpp"
#include "rosbag.hpp"
#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/version.h>
#include <spdlog/spdlog.h>
// stl
#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>

namespace rko_lio::ros::utils {
using point_cloud_interfaces::msg::CompressedPointCloud2;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

core::Vector3sVector point_cloud2_to_eigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  const size_t point_count = static_cast<size_t>(msg->height) * msg->width;
  core::Vector3sVector points;
  points.reserve(point_count);
  sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
  for (size_t i = 0; i < point_count; ++i, ++msg_x, ++msg_y, ++msg_z) {
    points.emplace_back(*msg_x, *msg_y, *msg_z);
  }
  return points;
}

RawScan point_cloud2_to_eigen_with_timestamps(const PointCloud2::ConstSharedPtr& msg) {
  using sensor_msgs::PointCloud2ConstIterator;
  // getting points and time in a single cycle loop
  const size_t point_count = static_cast<size_t>(msg->height) * msg->width;
  RawScan scan;
  scan.points.reserve(point_count);
  scan.timestamps.reserve(point_count);
  PointCloud2ConstIterator<float> msg_x(*msg, "x");
  PointCloud2ConstIterator<float> msg_y(*msg, "y");
  PointCloud2ConstIterator<float> msg_z(*msg, "z");

  const auto& timestamp_field = std::invoke([&msg]() -> PointField {
    for (const PointField& field : msg->fields) {
      if ((field.name == "t" || field.name == "time" || field.name == "timestamp" || field.name == "timestamps" ||
           field.name == "stamps") &&
          field.count != 0U) {
        return field;
      }
    }
    throw core::InputError(
        "No per-point timestamp field (t/time/timestamp/timestamps/stamps). Disable deskew, or add timestamps.");
  });

  // templated lambda (auto) ftw
  const auto extract_points_and_timestamps = [&](auto&& time_iter) {
    for (size_t i = 0; i < point_count; ++i, ++msg_x, ++msg_y, ++msg_z, ++time_iter) {
      scan.points.emplace_back(*msg_x, *msg_y, *msg_z);
      scan.timestamps.emplace_back(static_cast<double>(*time_iter));
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
    throw core::InputError("Unsupported timestamp field datatype " + std::to_string(timestamp_field.datatype) +
                           ". Please open an issue.");
  }

  return scan;
}

LidarDeserializer::LidarDeserializer(const std::string_view type) {
  if (type == rosidl_generator_traits::name<CompressedPointCloud2>()) {
    codec.emplace();
  }
}

PointCloud2::ConstSharedPtr LidarDeserializer::operator()(const std::shared_ptr<rclcpp::SerializedMessage>& msg) const {
  if (!codec) {
    return deserialize<PointCloud2>(*msg);
  }
  if (!decoder) {
    const std::string format = deserialize<CompressedPointCloud2>(*msg)->format;
    decoder = codec->getDecoderByName(format);
    if (!decoder) {
      throw core::InputError("No point_cloud_transport plugin named '" + format + "' is installed.");
    }
  }
  const auto decoded = decoder->decode(msg);
  if (!decoded) {
    spdlog::warn("Dropping scan: could not decode it: {}", decoded.error());
    return nullptr;
  }
  return decoded->value_or(nullptr);
}

rclcpp::SubscriptionBase::SharedPtr
create_lidar_subscription(const rclcpp::Node::SharedPtr& node,
                          const std::string& topic,
                          const rclcpp::QoS& qos,
                          const std::function<void(const PointCloud2::ConstSharedPtr&)>& callback) {
  auto publishers = node->get_publishers_info_by_topic(topic);
  if (publishers.empty()) {
    spdlog::info("Waiting for a publisher on {}.", topic);
  }
  while (publishers.empty()) {
    if (!rclcpp::sleep_for(std::chrono::milliseconds(100), node->get_node_base_interface()->get_context())) {
      return nullptr;
    }
    publishers = node->get_publishers_info_by_topic(topic);
  }
  const std::string& type = publishers.front().topic_type();
  if (type == rosidl_generator_traits::name<CompressedPointCloud2>()) {
    return node->create_generic_subscription(
        topic, type, qos,
#if RCLCPP_VERSION_MAJOR >= 28
        [deserialize_lidar = LidarDeserializer(type), callback](std::unique_ptr<rclcpp::SerializedMessage> msg) {
          if (const auto cloud = deserialize_lidar(std::move(msg))) {
#else
        [deserialize_lidar = LidarDeserializer(type), callback](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          if (const auto cloud = deserialize_lidar(msg)) {
#endif
            callback(cloud);
          }
        });
  }
  return node->create_subscription<PointCloud2>(topic, qos, callback);
}

} // namespace rko_lio::ros::utils
