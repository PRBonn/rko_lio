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
// Ported to rko_lio from kiss-icp (kiss_icp/cpp/kiss_icp/core/VoxelHashMap.{hpp,cpp}).

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

static const std::array<Voxel, 27> shifts{
    Voxel{-1, -1, -1}, Voxel{-1, -1, 0}, Voxel{-1, -1, 1}, //
    Voxel{-1, 0, -1},  Voxel{-1, 0, 0},  Voxel{-1, 0, 1},  //
    Voxel{-1, 1, -1},  Voxel{-1, 1, 0},  Voxel{-1, 1, 1},  //

    Voxel{0, -1, -1},  Voxel{0, -1, 0},  Voxel{0, -1, 1},  //
    Voxel{0, 0, -1},   Voxel{0, 0, 0},   Voxel{0, 0, 1},   //
    Voxel{0, 1, -1},   Voxel{0, 1, 0},   Voxel{0, 1, 1},   //

    Voxel{1, -1, -1},  Voxel{1, -1, 0},  Voxel{1, -1, 1}, //
    Voxel{1, 0, -1},   Voxel{1, 0, 0},   Voxel{1, 0, 1},  //
    Voxel{1, 1, -1},   Voxel{1, 1, 0},   Voxel{1, 1, 1}};

} // namespace

namespace rko_lio::core {

VoxelHashMap::VoxelHashMap(const double voxel_size,
                           const double clipping_distance,
                           const unsigned int max_points_per_voxel)
    : voxel_size_(voxel_size),
      inv_voxel_size_(1.0 / voxel_size),
      clipping_distance_(clipping_distance),
      max_points_per_voxel_(max_points_per_voxel) {}

std::tuple<Eigen::Vector3d, double> VoxelHashMap::get_closest_neighbor(const Eigen::Vector3d& query) const {
  Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
  double closest_distance = std::numeric_limits<double>::max();
  const Voxel voxel = point_to_voxel(query, inv_voxel_size_);
  std::for_each(shifts.cbegin(), shifts.cend(), [&](const Voxel& voxel_shift) {
    const Voxel query_voxel = voxel + voxel_shift;
    const auto search = map_.find(query_voxel);
    if (search != map_.end()) {
      const VoxelBlock& voxel_points = search.value();
      const Eigen::Vector3d& neighbor =
          *std::min_element(voxel_points.cbegin(), voxel_points.cend(), [&](const auto& lhs, const auto& rhs) {
            return (lhs - query).squaredNorm() < (rhs - query).squaredNorm();
          });
      double distance = (neighbor - query).norm();
      if (distance < closest_distance) {
        closest_neighbor = neighbor;
        closest_distance = distance;
      }
    }
  });
  return std::make_tuple(closest_neighbor, closest_distance);
}

void VoxelHashMap::add_points(const std::vector<Eigen::Vector3d>& points) {
  const double map_resolution_sq = voxel_size_ * voxel_size_ / max_points_per_voxel_;
  std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector3d& p) {
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

void VoxelHashMap::remove_points_far_from_location(const Eigen::Vector3d& origin) {
  const double clipping_distance_sq = clipping_distance_ * clipping_distance_;
  for (auto it = map_.begin(); it != map_.end();) {
    const VoxelBlock& voxel_points = it->second;
    if ((voxel_points.front() - origin).squaredNorm() > clipping_distance_sq) {
      it = map_.erase(it);
    } else {
      ++it;
    }
  }
}

void VoxelHashMap::update(const std::vector<Eigen::Vector3d>& points, const Sophus::SE3d& pose) {
  std::vector<Eigen::Vector3d> points_transformed(points.size());
  std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                 [&](const auto& point) { return pose * point; });
  const Eigen::Vector3d& origin = pose.translation();
  add_points(points_transformed);
  remove_points_far_from_location(origin);
}

std::vector<Eigen::Vector3d> VoxelHashMap::pointcloud() const {
  std::vector<Eigen::Vector3d> point_cloud;
  point_cloud.reserve(map_.size() * max_points_per_voxel_);
  for (const auto& [_, block] : map_) {
    point_cloud.insert(point_cloud.end(), block.cbegin(), block.cend());
  }
  return point_cloud;
}

} // namespace rko_lio::core
