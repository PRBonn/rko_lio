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
#include <sophus/se3.hpp>
#include <unordered_map>
#include <vector>

namespace {
using Voxel = Eigen::Vector3i;

inline Voxel PointToVoxel(const Eigen::Vector3d& point, const double voxel_size) {
  return {static_cast<int>(std::floor(point.x() / voxel_size)), static_cast<int>(std::floor(point.y() / voxel_size)),
          static_cast<int>(std::floor(point.z() / voxel_size))};
}
} // namespace

namespace rko_lio::core {
// if you need even better runtime-performance, consider using Luca Lobefaro's version of one cycle downsampling here:
// https://github.com/PRBonn/kiss-icp/pull/347
// although it does lead to worse odometry performance in certain situations

std::vector<Eigen::Vector3d> voxel_down_sample(const std::vector<Eigen::Vector3d>& frame, const double voxel_size) {
  std::unordered_map<Voxel, Eigen::Vector3d> grid;
  grid.reserve(frame.size());
  std::for_each(frame.cbegin(), frame.cend(), [&](const auto& point) {
    const auto voxel = PointToVoxel(point, voxel_size);
    if (!grid.contains(voxel)) {
      grid.insert({voxel, point});
    }
  });
  std::vector<Eigen::Vector3d> frame_dowsampled;
  frame_dowsampled.reserve(grid.size());
  std::for_each(grid.cbegin(), grid.cend(),
                [&](const auto& voxel_and_point) { frame_dowsampled.emplace_back(voxel_and_point.second); });
  return frame_dowsampled;
}

} // namespace rko_lio::core
