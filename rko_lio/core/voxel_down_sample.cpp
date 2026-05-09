// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
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

#include "voxel_down_sample.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <functional>
#include <tsl/robin_set.h>
#include <utility>
#include <vector>

namespace rko_lio::core {

std::vector<Eigen::Vector3d> voxel_down_sample(const std::vector<Eigen::Vector3d>& frame, const double voxel_size) {
  const double inv_voxel_size = 1.0 / voxel_size;
  tsl::robin_set<Eigen::Vector3i> seen;
  seen.reserve(frame.size());
  std::vector<Eigen::Vector3d> frame_downsampled;
  frame_downsampled.reserve(frame.size());
  for (const auto& point : frame) {
    if (seen.insert(point_to_voxel(point, inv_voxel_size)).second) {
      frame_downsampled.emplace_back(point);
    }
  }
  return frame_downsampled;
}

std::vector<Eigen::Vector3d> voxel_down_sample_sorted(const std::vector<Eigen::Vector3d>& frame,
                                                     const double voxel_size) {
  const double inv_voxel_size = 1.0 / voxel_size;
  tsl::robin_set<Eigen::Vector3i> seen;
  seen.reserve(frame.size());
  std::vector<std::pair<std::size_t, Eigen::Vector3d>> hashed;
  hashed.reserve(frame.size());
  const std::hash<Eigen::Vector3i> hasher{};
  for (const auto& point : frame) {
    const auto voxel = point_to_voxel(point, inv_voxel_size);
    if (seen.insert(voxel).second) {
      hashed.emplace_back(hasher(voxel), point);
    }
  }
  std::sort(hashed.begin(), hashed.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  std::vector<Eigen::Vector3d> frame_downsampled;
  frame_downsampled.reserve(hashed.size());
  for (auto& [_, v] : hashed) {
    frame_downsampled.emplace_back(std::move(v));
  }
  return frame_downsampled;
}

} // namespace rko_lio::core
