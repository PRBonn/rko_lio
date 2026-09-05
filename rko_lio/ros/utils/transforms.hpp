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
#include <rko_lio/core/error.hpp>
#include <rko_lio/core/util.hpp>

#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <vector>

namespace rko_lio::ros::utils {
inline geometry_msgs::msg::Pose sophus_to_pose(const Sophus::SE3s& T) {
  geometry_msgs::msg::Pose t;
  t.position.x = T.translation().x();
  t.position.y = T.translation().y();
  t.position.z = T.translation().z();

  Eigen::Quaternions q(T.so3().unit_quaternion());
  t.orientation.x = q.x();
  t.orientation.y = q.y();
  t.orientation.z = q.z();
  t.orientation.w = q.w();

  return t;
}

inline geometry_msgs::msg::Transform sophus_to_transform(const Sophus::SE3s& T) {
  geometry_msgs::msg::Transform t;
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();

  Eigen::Quaternions q(T.so3().unit_quaternion());
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();

  return t;
}

inline Sophus::SE3s transform_to_sophus(const geometry_msgs::msg::TransformStamped& transform) {
  const auto& t = transform.transform;
  return {Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).cast<core::Scalar>(),
          Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z).cast<core::Scalar>()};
}

// [qx, qy, qz, qw, tx, ty, tz]. Throws core::InputError on any other size or a degenerate quaternion.
inline Sophus::SE3s to_se3(const std::vector<double>& quat_xyzw_xyz) {
  if (quat_xyzw_xyz.size() != 7) {
    throw core::InputError("Expected 7 values (qx, qy, qz, qw, x, y, z) but got " +
                           std::to_string(quat_xyzw_xyz.size()) + ".");
  }
  const Eigen::Quaterniond quat(quat_xyzw_xyz.at(3), quat_xyzw_xyz.at(0), quat_xyzw_xyz.at(1), quat_xyzw_xyz.at(2));
  if (quat.norm() < 1e-6) {
    throw core::InputError("Quaternion is degenerate (norm is close to zero).");
  }
  return {quat.normalized().cast<core::Scalar>(),
          Eigen::Vector3d(quat_xyzw_xyz.at(4), quat_xyzw_xyz.at(5), quat_xyzw_xyz.at(6)).cast<core::Scalar>()};
}

inline std::optional<Sophus::SE3s> get_transform(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                                 const std::string& from_frame,
                                                 const std::string& to_frame,
                                                 const std::chrono::nanoseconds time,
                                                 const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(0)) {
  geometry_msgs::msg::TransformStamped from_to_transform;
  const tf2::TimePoint tf_time{time};
  const tf2::Duration tf_timeout{timeout};
  try {
    tf_buffer->_validateFrameId("from_frame", from_frame);
    tf_buffer->_validateFrameId("to frame", to_frame);
    const std::unique_ptr<std::string> error_str = std::make_unique<std::string>();
    if (!tf_buffer->canTransform(to_frame, from_frame, tf_time, tf_timeout, error_str.get())) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"),
                         "Cannot transform from: " << from_frame << " -> to: " << to_frame
                                                   << " at time: " << time.count() << "ns because of: " << *error_str);
      return std::nullopt;
    }
    from_to_transform = tf_buffer->lookupTransform(to_frame, from_frame, tf_time);
    return transform_to_sophus(from_to_transform);
  } catch (const tf2::InvalidArgumentException& e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"),
                       "TF lookup error (InvalidArgumentException): " << e.what());
    RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"),
                       "Arguments are, to_frame: " << to_frame << ", from_frame: " << from_frame
                                                   << ", time(ns): " << time.count());
  } catch (const tf2::LookupException& e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("transform lookup"), "TF lookup error (LookupException): " << e.what());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("transform lookup"),
                        "Could not get the transform from: " << from_frame << " to " << to_frame << ": " << ex.what());
  }
  return std::nullopt;
}
} // namespace rko_lio::ros::utils
