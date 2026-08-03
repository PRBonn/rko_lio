// MIT License
//
// Copyright (c) 2025 Meher V.R. Malladi.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once
#include <rko_lio/core/util.hpp>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <type_traits>

namespace rko_lio::tests {

constexpr bool IS_DOUBLE = std::is_same_v<core::Scalar, double>;

// double has tighter tolerances when being used. float is looser
constexpr double LOOSE_TOL = IS_DOUBLE ? 1e-6 : 1e-4;
constexpr double TOL = IS_DOUBLE ? 1e-9 : 1e-4;
constexpr double EXACT_TOL = IS_DOUBLE ? 1e-12 : 1e-4;

inline bool approx_equal(const Eigen::Vector3s& a, const Eigen::Vector3s& b, double tolerance = LOOSE_TOL) {
  return (a - b).norm() < tolerance;
}

inline bool approx_equal(const Sophus::SO3s& a, const Sophus::SO3s& b, double tolerance = LOOSE_TOL) {
  return (a.inverse() * b).log().norm() < tolerance;
}

inline bool approx_equal(const Sophus::SE3s& a, const Sophus::SE3s& b, double tolerance = LOOSE_TOL) {
  return (a.inverse() * b).log().norm() < tolerance;
}

} // namespace rko_lio::tests
