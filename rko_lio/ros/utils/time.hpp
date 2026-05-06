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
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <rclcpp/time.hpp>

namespace rko_lio::ros::utils {
inline std::chrono::nanoseconds to_ns(const builtin_interfaces::msg::Time& stamp) {
  return std::chrono::nanoseconds(rclcpp::Time(stamp).nanoseconds());
}

inline std::chrono::nanoseconds to_ns(const rclcpp::Time& time) {
  return std::chrono::nanoseconds(time.nanoseconds());
}

inline builtin_interfaces::msg::Time to_ros_time(std::chrono::nanoseconds time) {
  return rclcpp::Time(time.count());
}
} // namespace rko_lio::ros::utils
