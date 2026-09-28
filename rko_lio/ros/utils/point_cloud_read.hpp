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

#pragma once
#include <rko_lio/core/util.hpp>

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <optional>
#include <sophus/se3.hpp>
#include <string>
#include <string_view>
// ros
#include <point_cloud_transport/point_cloud_codec.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rko_lio::ros::utils {
core::Vector3sVector point_cloud2_to_eigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

struct RawScan {
  core::Vector3sVector points;
  std::vector<double> timestamps;
};

RawScan point_cloud2_to_eigen_with_timestamps(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

struct LidarDeserializer {
  explicit LidarDeserializer(std::string_view type);
  // nullptr if the compressed cloud does not decode
  sensor_msgs::msg::PointCloud2::ConstSharedPtr operator()(const std::shared_ptr<rclcpp::SerializedMessage>& msg) const;
  mutable std::optional<point_cloud_transport::PointCloudCodec> codec;
  mutable std::shared_ptr<point_cloud_transport::SubscriberPlugin> decoder;
};

// Waits until `topic` has a publisher, then subscribes as the type that publisher advertises.
// nullptr if ROS shuts down while waiting.
rclcpp::SubscriptionBase::SharedPtr
create_lidar_subscription(const rclcpp::Node::SharedPtr& node,
                          const std::string& topic,
                          const rclcpp::QoS& qos,
                          const std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)>& callback);
}; // namespace rko_lio::ros::utils
