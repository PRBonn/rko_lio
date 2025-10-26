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

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>

namespace rko_lio::ros::utils {
inline geometry_msgs::msg::Pose sophus_to_pose(const Sophus::SE3d& T) {
  geometry_msgs::msg::Pose t;
  t.position.x = T.translation().x();
  t.position.y = T.translation().y();
  t.position.z = T.translation().z();

  Eigen::Quaterniond q(T.so3().unit_quaternion());
  t.orientation.x = q.x();
  t.orientation.y = q.y();
  t.orientation.z = q.z();
  t.orientation.w = q.w();

  return t;
}

inline geometry_msgs::msg::Transform sophus_to_transform(const Sophus::SE3d& T) {
  geometry_msgs::msg::Transform t;
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.so3().unit_quaternion());
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();

  return t;
}

template <typename Scalar = double>
inline Sophus::SE3<Scalar> transform_to_sophus(const geometry_msgs::msg::TransformStamped& transform) {
  const auto& t = transform.transform;
  return {typename Sophus::SE3<Scalar>::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
          typename Sophus::SE3<Scalar>::Point(t.translation.x, t.translation.y, t.translation.z)};
}

template <typename Scalar = double>
std::optional<Sophus::SE3<Scalar>>
get_transform(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
              const std::string& from_frame,
              const std::string& to_frame,
              const std::chrono::duration<double>& time,
              const std::chrono::duration<double>& timeout = std::chrono::duration<double>(0)) {
  geometry_msgs::msg::TransformStamped from_to_transform;
  try {
    tf_buffer->_validateFrameId("from_frame", from_frame);
    tf_buffer->_validateFrameId("to frame", to_frame);
    std::unique_ptr<std::string> error_str = std::make_unique<std::string>();
    if (!tf_buffer->canTransform(to_frame, from_frame, tf2::TimePoint(std::chrono::duration_cast<tf2::IDuration>(time)),
                                 std::chrono::duration_cast<tf2::Duration>(timeout), error_str.get())) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"),
                         "Cannot transfrom from: " << from_frame << " -> to: " << to_frame
                                                   << " at time: " << time.count() << " because of: " << *error_str);
      return std::nullopt;
    }
    from_to_transform = tf_buffer->lookupTransform(to_frame, from_frame,
                                                   tf2::TimePoint(std::chrono::duration_cast<tf2::Duration>(time)));
    return transform_to_sophus<Scalar>(from_to_transform);
  } catch (const tf2::InvalidArgumentException& e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"),
                       "TF lookup error (InvalidArgumentException): " << e.what());
    RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"),
                       "Arguments are, to_frame: " << to_frame << ", from_frame: " << from_frame
                                                   << ", time: " << time.count());
  } catch (const tf2::LookupException& e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"), "TF lookup error (LookupException): " << e.what());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("transform lookup"),
                        "Could not get the transform from: " << from_frame << " to " << to_frame << ": " << ex.what());
  }
  return std::nullopt;
}
} // namespace rko_lio::ros::utils
