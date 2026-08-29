/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 * Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
 * Stachniss.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once

// brings VoxelHash and point_to_voxel
#include "voxel_down_sample.hpp"

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <sophus/se3.hpp>
#include <stdexcept>
#include <tsl/robin_map.h>
#include <tuple>
#include <vector>

namespace rko_lio::core {

using Voxel = Eigen::Vector3i;

/// A map point is a signed offset from the centre of its voxel, quantized to int8
struct VoxelBlock {
  static constexpr unsigned int max_points = 8;
  using PointArray = std::array<Eigen::Vector3i8, max_points>;

  PointArray points{};
  std::uint8_t size = 0;

  bool full() const { return size == max_points; }

  void push_back(const Eigen::Vector3i8& offset) {
    if (full()) {
      throw std::out_of_range("VoxelBlock is full, check full() before pushing");
    }
    *std::next(points.begin(), size) = offset;
    ++size;
  }

  PointArray::const_iterator begin() const { return points.begin(); }
  PointArray::const_iterator end() const { return points.begin() + size; } // capacity is 8, size is what's filled
};

class VoxelHashMap {
public:
  explicit VoxelHashMap(const Scalar voxel_size, const Scalar clipping_distance);

  void clear() { voxels_.clear(); }
  bool empty() const { return voxels_.empty(); }
  void update(const std::vector<Eigen::Vector3s>& points, const Sophus::SE3s& pose);
  void add_points(const std::vector<Eigen::Vector3s>& points, const Sophus::SE3s& pose);
  void remove_points_far_from_location(const Eigen::Vector3s& location);
  std::vector<Eigen::Vector3s> points() const;

  /// Nearest point to `query` strictly within `max_distance`, or nullopt if there is none.
  std::optional<Eigen::Vector3s> get_closest_neighbor(const Eigen::Vector3s& query, const Scalar max_distance) const;

private:
  Scalar voxel_size_;
  Scalar inv_voxel_size_;
  Scalar quantum_;
  Scalar inv_quantum_;
  Scalar clipping_distance_;
  tsl::robin_map<Voxel, VoxelBlock, VoxelHash> voxels_;
};

} // namespace rko_lio::core
