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
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <sophus/se3.hpp>

namespace rko_lio::core {
// single cycle voxel downsample, see https://github.com/PRBonn/kiss-icp/pull/347
std::vector<Eigen::Vector3d> voxel_down_sample(const std::vector<Eigen::Vector3d>& frame, const double voxel_size);

// like voxel_down_sample but output sorted by hash(voxel). used when the output feeds another downsample pass.
// the spatial hash scatters adjacent voxels far apart, so the next pass's first-write-wins per voxel sees a
// diverse pick of points instead of one biased by the lidar sweep order, essentially breaking that regular pattern.
// leads to an improvement in registration performance, see https://github.com/PRBonn/rko_lio/pull/136
std::vector<Eigen::Vector3d> voxel_down_sample_sorted(const std::vector<Eigen::Vector3d>& frame,
                                                      const double voxel_size);

inline Eigen::Vector3i point_to_voxel(const Eigen::Vector3d& point, const double inv_voxel_size) {
  return {static_cast<int>(std::floor(point.x() * inv_voxel_size)),
          static_cast<int>(std::floor(point.y() * inv_voxel_size)),
          static_cast<int>(std::floor(point.z() * inv_voxel_size))};
}
} // namespace rko_lio::core

template <>
struct std::hash<Eigen::Vector3i> {
  std::size_t operator()(const Eigen::Vector3i& voxel) const {
    const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
  }
};
