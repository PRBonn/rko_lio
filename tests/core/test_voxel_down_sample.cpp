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

#include "rko_lio/core/voxel_down_sample.hpp"
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <tuple>
#include <vector>

using rko_lio::core::voxel_down_sample;

namespace {
std::set<std::tuple<int, int, int>> as_voxel_set(const std::vector<Eigen::Vector3d>& points, double voxel_size) {
  std::set<std::tuple<int, int, int>> out;
  for (const auto& p : points) {
    out.emplace(static_cast<int>(std::floor(p.x() / voxel_size)),
                static_cast<int>(std::floor(p.y() / voxel_size)),
                static_cast<int>(std::floor(p.z() / voxel_size)));
  }
  return out;
}
} // namespace

TEST_CASE("voxel_down_sample: empty input -> empty output", "[voxel_down_sample]") {
  const std::vector<Eigen::Vector3d> empty;
  const auto result = voxel_down_sample(empty, 0.5);
  REQUIRE(result.empty());
}

TEST_CASE("voxel_down_sample: all points within one voxel -> 1 output", "[voxel_down_sample]") {
  const double voxel_size = 1.0;
  const std::vector<Eigen::Vector3d> points = {
      {0.1, 0.1, 0.1}, {0.2, 0.3, 0.4}, {0.5, 0.5, 0.5}, {0.99, 0.99, 0.99}};
  const auto result = voxel_down_sample(points, voxel_size);
  REQUIRE(result.size() == 1);
}

TEST_CASE("voxel_down_sample: N well-separated points -> N outputs", "[voxel_down_sample]") {
  const double voxel_size = 0.5;
  const std::vector<Eigen::Vector3d> points = {
      {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 10.0}, {10.0, 10.0, 10.0}};
  const auto result = voxel_down_sample(points, voxel_size);
  REQUIRE(result.size() == points.size());
}

TEST_CASE("voxel_down_sample: order independence", "[voxel_down_sample]") {
  const double voxel_size = 0.5;
  std::vector<Eigen::Vector3d> a = {{0.1, 0.1, 0.1}, {3.2, 1.1, 0.7}, {7.5, 4.5, 2.3}, {0.4, 0.4, 0.4}};
  std::vector<Eigen::Vector3d> b = a;
  std::reverse(b.begin(), b.end());

  const auto ra = voxel_down_sample(a, voxel_size);
  const auto rb = voxel_down_sample(b, voxel_size);

  // The picked representative may differ per voxel; the set of occupied voxels must not.
  REQUIRE(as_voxel_set(ra, voxel_size) == as_voxel_set(rb, voxel_size));
  REQUIRE(ra.size() == rb.size());
}

TEST_CASE("voxel_down_sample: doubling voxel size collapses points", "[voxel_down_sample]") {
  const std::vector<Eigen::Vector3d> points = {{0.1, 0.1, 0.1}, {0.7, 0.1, 0.1}};
  const auto small = voxel_down_sample(points, 0.5);
  const auto large = voxel_down_sample(points, 1.0);
  REQUIRE(small.size() == 2);
  REQUIRE(large.size() == 1);
}
