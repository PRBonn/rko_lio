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
#include <vector>

namespace rko_lio::tests {

// Hollow cube of side 2*half_extent centered at the origin, six faces sampled at
// `spacing`. 
inline std::vector<Eigen::Vector3d> make_hollow_cube(double spacing = 0.5, double half_extent = 5.0) {
  std::vector<Eigen::Vector3d> points;
  const int n = static_cast<int>(2 * half_extent / spacing) + 1;
  points.reserve(static_cast<size_t>(6 * n * n));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double a = -half_extent + i * spacing;
      const double b = -half_extent + j * spacing;
      points.emplace_back(half_extent, a, b);
      points.emplace_back(-half_extent, a, b);
      points.emplace_back(a, half_extent, b);
      points.emplace_back(a, -half_extent, b);
      points.emplace_back(a, b, half_extent);
      points.emplace_back(a, b, -half_extent);
    }
  }
  return points;
}

} // namespace rko_lio::tests