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
//
// NOTE: This implementation is heavily inspired in the original CT-ICP VoxelHashMap implementation,
// although it was heavily modifed and drastically simplified, but if you are using this module you
// should at least acknoowledge the work from CT-ICP by giving a star on GitHub.
//
// Modified from kiss-icp (kiss_icp/cpp/kiss_icp/core/VoxelHashMap.{hpp,cpp}).

#include "voxel_hash_map.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

namespace {
using rko_lio::core::Voxel;

// the order of the shifts is deliberate
static const std::array<Voxel, 27> shifts{
    Voxel{0, 0, 0}, // home

    Voxel{-1, 0, 0},   Voxel{0, -1, 0},  Voxel{0, 0, -1}, // faces
    Voxel{0, 0, 1},    Voxel{0, 1, 0},   Voxel{1, 0, 0},

    Voxel{-1, -1, 0},  Voxel{-1, 0, -1}, Voxel{-1, 0, 1}, // edges
    Voxel{-1, 1, 0},   Voxel{0, -1, -1}, Voxel{0, -1, 1},  Voxel{0, 1, -1}, Voxel{0, 1, 1},
    Voxel{1, -1, 0},   Voxel{1, 0, -1},  Voxel{1, 0, 1},   Voxel{1, 1, 0},

    Voxel{-1, -1, -1}, Voxel{-1, -1, 1}, Voxel{-1, 1, -1}, // corners
    Voxel{-1, 1, 1},   Voxel{1, -1, -1}, Voxel{1, -1, 1},  Voxel{1, 1, -1}, Voxel{1, 1, 1}};

} // namespace

namespace rko_lio::core {

VoxelHashMap::VoxelHashMap(const Scalar voxel_size,
                           const Scalar clipping_distance,
                           const unsigned int max_points_per_voxel)
    : voxel_size_(voxel_size),
      inv_voxel_size_(1.0 / voxel_size),
      clipping_distance_(clipping_distance),
      max_points_per_voxel_(max_points_per_voxel) {}

std::optional<VoxelBlock::const_iterator> VoxelHashMap::get_closest_neighbor(const Eigen::Vector3s& query,
                                                                             const Scalar max_distance) const {
  Scalar closest_distance_sq = max_distance * max_distance;
  std::optional<VoxelBlock::const_iterator> closest;
  const Voxel voxel = point_to_voxel(query, inv_voxel_size_);

  // lower_bounds is squared distance from the query to the near face of a neighbouring voxel, per axis.
  // Columns are indexed by voxel shift + 1, rows are axes
  const Eigen::Vector3s offset = query - voxel.cast<Scalar>() * voxel_size_;
  const Eigen::Vector3s to_far = Eigen::Vector3s::Constant(voxel_size_) - offset;
  Eigen::Matrix3s lower_bounds;
  lower_bounds.col(0) = offset.array().square();
  lower_bounds.col(1).setZero();
  lower_bounds.col(2) = to_far.array().square();

  for (const Voxel& shift : shifts) {
    // lower bound on the distance to query within this voxel + shift
    const Scalar lower_bound_sq =
        lower_bounds(0, shift.x() + 1) + lower_bounds(1, shift.y() + 1) + lower_bounds(2, shift.z() + 1);
    if (lower_bound_sq >= closest_distance_sq) {
      continue;
    }
    const auto search = map_.find(voxel + shift);
    if (search == map_.end()) {
      continue;
    }
    // returning an iterator rather than a copy reduces branching (with the usual compiler caveats)
    const VoxelBlock& voxel_points = search.value();
    for (auto neighbor = voxel_points.cbegin(); neighbor != voxel_points.cend(); ++neighbor) {
      const Scalar distance_sq = (*neighbor - query).squaredNorm();
      if (distance_sq < closest_distance_sq) {
        closest_distance_sq = distance_sq;
        closest = neighbor;
      }
    }
  }
  return closest;
}

void VoxelHashMap::add_points(const std::vector<Eigen::Vector3s>& points, const Sophus::SE3s& pose) {
  const Scalar map_resolution_sq = voxel_size_ * voxel_size_ / max_points_per_voxel_;
  std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector3s& point) {
    const Eigen::Vector3s p = pose * point;
    const Voxel voxel = point_to_voxel(p, inv_voxel_size_);
    auto it = map_.try_emplace(voxel).first;
    VoxelBlock& voxel_points = it.value();
    if (voxel_points.size() == max_points_per_voxel_ ||
        std::any_of(voxel_points.cbegin(), voxel_points.cend(),
                    [&](const auto& voxel_point) { return (voxel_point - p).squaredNorm() < map_resolution_sq; })) {
      return;
    }
    voxel_points.reserve(max_points_per_voxel_);
    voxel_points.emplace_back(p);
  });
}

void VoxelHashMap::remove_points_far_from_location(const Eigen::Vector3s& origin) {
  const Scalar clipping_distance_sq = clipping_distance_ * clipping_distance_;
  for (auto it = map_.begin(); it != map_.end();) {
    const VoxelBlock& voxel_points = it->second;
    if ((voxel_points.front() - origin).squaredNorm() > clipping_distance_sq) {
      it = map_.erase(it);
    } else {
      ++it;
    }
  }
}

void VoxelHashMap::update(const std::vector<Eigen::Vector3s>& points, const Sophus::SE3s& pose) {
  add_points(points, pose);
  remove_points_far_from_location(pose.translation());
}

std::vector<Eigen::Vector3s> VoxelHashMap::pointcloud() const {
  std::vector<Eigen::Vector3s> point_cloud;
  point_cloud.reserve(map_.size() * max_points_per_voxel_);
  for (const auto& [_, block] : map_) {
    point_cloud.insert(point_cloud.end(), block.cbegin(), block.cend());
  }
  return point_cloud;
}

} // namespace rko_lio::core
