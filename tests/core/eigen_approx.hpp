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
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace rko_lio::tests {

inline bool approx_equal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol = 1e-6) {
  return (a - b).norm() < tol;
}

inline bool approx_equal(const Sophus::SO3d& a, const Sophus::SO3d& b, double tol = 1e-6) {
  return (a.inverse() * b).log().norm() < tol;
}

inline bool approx_equal(const Sophus::SE3d& a, const Sophus::SE3d& b, double tol = 1e-6) {
  return (a.inverse() * b).log().norm() < tol;
}

} // namespace rko_lio::tests
