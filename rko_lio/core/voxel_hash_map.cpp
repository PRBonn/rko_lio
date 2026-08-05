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
// Modified heavily from kiss-icp (kiss_icp/cpp/kiss_icp/core/VoxelHashMap.{hpp,cpp}).

#include "voxel_hash_map.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <sophus/se3.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {
using rko_lio::core::Scalar;
using rko_lio::core::Voxel;

// the order of the shifts is deliberate
const std::array<Voxel, 27> shifts{
    Voxel{0, 0, 0}, // home

    Voxel{-1, 0, 0},   Voxel{0, -1, 0},  Voxel{0, 0, -1}, // faces
    Voxel{0, 0, 1},    Voxel{0, 1, 0},   Voxel{1, 0, 0},

    Voxel{-1, -1, 0},  Voxel{-1, 0, -1}, Voxel{-1, 0, 1}, // edges
    Voxel{-1, 1, 0},   Voxel{0, -1, -1}, Voxel{0, -1, 1},  Voxel{0, 1, -1}, Voxel{0, 1, 1},
    Voxel{1, -1, 0},   Voxel{1, 0, -1},  Voxel{1, 0, 1},   Voxel{1, 1, 0},

    Voxel{-1, -1, -1}, Voxel{-1, -1, 1}, Voxel{-1, 1, -1}, // corners
    Voxel{-1, 1, 1},   Voxel{1, -1, -1}, Voxel{1, -1, 1},  Voxel{1, 1, -1}, Voxel{1, 1, 1}};

// Fixed point layout. A voxel is cut into QUANTA_PER_VOXEL steps per axis and a point is stored as its offset
// from the centre in those steps, spanning [-127, 127], i.e. int8_t. One step is voxel_size / QUANTA_PER_VOXEL m.
constexpr Scalar MAX_OFFSET = std::numeric_limits<std::int8_t>::max();
constexpr Scalar QUANTA_PER_VOXEL = 2 * MAX_OFFSET;
// past this a quantum is coarser than ~2 cm, which is the order of lidar range noise
constexpr Scalar MAX_VOXEL_SIZE = 5.0;

/// metres -> quanta
inline Eigen::Vector3s to_quanta(const Eigen::Vector3s& metres, const Scalar inv_quantum) {
  return metres * inv_quantum;
}

/// corner of `voxel`, in metres
inline Eigen::Vector3s voxel_corner(const Voxel& voxel, const Scalar voxel_size) {
  return voxel.cast<Scalar>() * voxel_size;
}

/// centre of `voxel`, in metres
inline Eigen::Vector3s voxel_centre(const Voxel& voxel, const Scalar voxel_size) {
  return (voxel.cast<Scalar>() + Eigen::Vector3s::Constant(Scalar(0.5))) * voxel_size;
}

/// `point` from the voxel corner, in quanta. Runs 0 to QUANTA_PER_VOXEL
inline Eigen::Vector3s quanta_from_corner(const Eigen::Vector3s& point,
                                          const Voxel& voxel,
                                          const Scalar voxel_size,
                                          const Scalar inv_quantum) {
  return to_quanta(point - voxel_corner(voxel, voxel_size), inv_quantum);
}

/// `point` from the voxel centre, in quanta. Stored offsets live in this frame
inline Eigen::Vector3s quanta_from_centre(const Eigen::Vector3s& point,
                                          const Voxel& voxel,
                                          const Scalar voxel_size,
                                          const Scalar inv_quantum) {
  return to_quanta(point - voxel_centre(voxel, voxel_size), inv_quantum);
}

/// re-origin quanta from the corner to the centre. Half a voxel is MAX_OFFSET quanta
inline Eigen::Vector3s corner_to_centre(const Eigen::Vector3s& quanta) {
  return quanta - Eigen::Vector3s::Constant(MAX_OFFSET);
}

/// against the centre of the voxel one `shift` away. A shift is a whole voxel, so QUANTA_PER_VOXEL per axis
inline Eigen::Vector3s quanta_from_neighbour_centre(const Eigen::Vector3s& quanta, const Voxel& shift) {
  return quanta - (shift.cast<Scalar>() * QUANTA_PER_VOXEL);
}

/// squared distance from the query to the near face of a neighbouring voxel, per axis.
/// Columns are indexed by voxel shift + 1, rows are axes.
inline Eigen::Matrix3s face_bounds(const Eigen::Vector3s& from_corner) {
  const Eigen::Vector3s from_high = Eigen::Vector3s::Constant(QUANTA_PER_VOXEL) - from_corner;
  Eigen::Matrix3s bounds;
  bounds.col(0) = from_corner.array().square();
  bounds.col(1).setZero();
  bounds.col(2) = from_high.array().square();
  return bounds;
}

/// round Scalar into int8 for storage. clamp prevents wrapping on type casts
inline Eigen::Vector3i8 to_stored_offset(const Eigen::Vector3s& quanta) {
  return quanta.array().round().min(MAX_OFFSET).max(-MAX_OFFSET).matrix().cast<std::int8_t>();
}
} // namespace

namespace rko_lio::core {

VoxelHashMap::VoxelHashMap(const Scalar voxel_size, const Scalar clipping_distance)
    : voxel_size_(voxel_size),
      inv_voxel_size_(static_cast<Scalar>(1.0 / voxel_size)),
      quantum_(voxel_size / QUANTA_PER_VOXEL),
      inv_quantum_(QUANTA_PER_VOXEL / voxel_size),
      clipping_distance_(clipping_distance) {
  if (voxel_size <= 0 || voxel_size > MAX_VOXEL_SIZE) {
    throw std::invalid_argument("voxel_size must be in (0, " + std::to_string(MAX_VOXEL_SIZE) + "] metres");
  }
}

std::optional<Eigen::Vector3s> VoxelHashMap::get_closest_neighbor(const Eigen::Vector3s& query,
                                                                  const Scalar max_distance) const {
  const Voxel voxel = point_to_voxel(query, inv_voxel_size_);
  const Eigen::Vector3s query_quanta_from_corner = quanta_from_corner(query, voxel, voxel_size_, inv_quantum_);
  const Eigen::Vector3s query_quanta_from_centre = corner_to_centre(query_quanta_from_corner);
  const Eigen::Matrix3s lower_bounds = face_bounds(query_quanta_from_corner);

  Scalar closest_distance_sq = square(max_distance * inv_quantum_);
  const Eigen::Vector3i8* closest_offset = nullptr;
  const Voxel* closest_shift = shifts.begin();

  for (const Voxel* shift = shifts.begin(); shift != shifts.end(); ++shift) {
    // lower bound on the distance to query within this voxel + shift
    const Scalar lower_bound_sq =
        lower_bounds(0, shift->x() + 1) + lower_bounds(1, shift->y() + 1) + lower_bounds(2, shift->z() + 1);
    if (lower_bound_sq >= closest_distance_sq) {
      continue;
    }
    const auto search = map_.find(voxel + *shift);
    if (search == map_.end()) {
      continue;
    }
    const Eigen::Vector3s query_quanta_from_neighbour = quanta_from_neighbour_centre(query_quanta_from_centre, *shift);
    const VoxelBlock& block = search.value();
    for (const Eigen::Vector3i8* neighbor = block.begin(); neighbor != block.end(); ++neighbor) {
      const Scalar distance_sq = (neighbor->cast<Scalar>() - query_quanta_from_neighbour).squaredNorm();
      if (distance_sq < closest_distance_sq) {
        closest_distance_sq = distance_sq;
        closest_offset = neighbor;
        closest_shift = shift;
      }
    }
  }

  if (closest_offset == nullptr) {
    return std::nullopt;
  }
  const Eigen::Vector3s query_quanta_from_neighbour =
      quanta_from_neighbour_centre(query_quanta_from_centre, *closest_shift);
  return query + (closest_offset->cast<Scalar>() - query_quanta_from_neighbour) * quantum_;
}

void VoxelHashMap::add_points(const std::vector<Eigen::Vector3s>& points, const Sophus::SE3s& pose) {
  // same radius as voxel_size^2 / max_points, but in quanta
  const Scalar map_resolution_sq = square(QUANTA_PER_VOXEL) / static_cast<Scalar>(VoxelBlock::max_points);
  std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector3s& point) {
    const Eigen::Vector3s p = pose * point;
    const Voxel voxel = point_to_voxel(p, inv_voxel_size_);
    // quantise before the spacing test, so the test sees the offset that would actually be stored
    const Eigen::Vector3i8 offset = to_stored_offset(quanta_from_centre(p, voxel, voxel_size_, inv_quantum_));
    auto it = map_.try_emplace(voxel).first;
    VoxelBlock& voxel_points = it.value();
    if (voxel_points.full() ||
        std::any_of(voxel_points.begin(), voxel_points.end(), [&](const Eigen::Vector3i8& voxel_point) {
          return (voxel_point.cast<Scalar>() - offset.cast<Scalar>()).squaredNorm() < map_resolution_sq;
        })) {
      return;
    }
    voxel_points.push_back(offset);
  });
}

void VoxelHashMap::remove_points_far_from_location(const Eigen::Vector3s& origin) {
  // a centre is within half a diagonal of every point it holds, so widening by that keeps a superset
  const Scalar half_diagonal = Scalar(0.5) * std::numbers::sqrt3_v<Scalar> * voxel_size_;
  const Scalar max_distance_sq = square(clipping_distance_ + half_diagonal);
  for (auto it = map_.begin(); it != map_.end();) {
    it = (voxel_centre(it->first, voxel_size_) - origin).squaredNorm() > max_distance_sq ? map_.erase(it)
                                                                                         : std::next(it);
  }
}

void VoxelHashMap::update(const std::vector<Eigen::Vector3s>& points, const Sophus::SE3s& pose) {
  add_points(points, pose);
  remove_points_far_from_location(pose.translation());
}

std::vector<Eigen::Vector3s> VoxelHashMap::pointcloud() const {
  std::vector<Eigen::Vector3s> point_cloud;
  point_cloud.reserve(map_.size() * VoxelBlock::max_points);
  for (const auto& [voxel, block] : map_) {
    const Eigen::Vector3s centre = voxel_centre(voxel, voxel_size_);
    for (const Eigen::Vector3i8& offset : block) {
      point_cloud.emplace_back(centre + offset.cast<Scalar>() * quantum_);
    }
  }
  return point_cloud;
}

} // namespace rko_lio::core
