#include "rko_lio/core/voxel_hash_map.hpp"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <vector>

using rko_lio::core::Scalar;
using rko_lio::core::Voxel;
using rko_lio::core::VoxelHashMap;

namespace {
constexpr Scalar kVoxelSize = 1.0;

Voxel voxel_of(const Eigen::Vector3s& point) {
  return {static_cast<int>(std::floor(point.x() / kVoxelSize)), static_cast<int>(std::floor(point.y() / kVoxelSize)),
          static_cast<int>(std::floor(point.z() / kVoxelSize))};
}
} // namespace

TEST_CASE("voxel_hash_map: a stored point reads back inside its own voxel", "[voxel_hash_map]") {
  VoxelHashMap map(kVoxelSize, 100.0);
  // one point per voxel, each just short of the voxel's upper face, where a stored point on the face would be filed
  // under the next voxel
  std::vector<Eigen::Vector3s> points;
  for (int i = -3; i < 3; ++i) {
    const auto corner = static_cast<Scalar>(i) * kVoxelSize;
    points.emplace_back(corner + static_cast<Scalar>(0.9999) * kVoxelSize, corner + static_cast<Scalar>(0.5),
                        corner + static_cast<Scalar>(0.9999) * kVoxelSize);
  }
  map.add_points(points, Sophus::SE3s());
  REQUIRE(map.voxels.size() == points.size());

  for (const auto& [voxel, block] : map.voxels) {
    for (const Eigen::Vector3i8& offset : block) {
      const Eigen::Vector3s stored = map.center_of_voxel(voxel) + offset.cast<Scalar>() * map.quantum;
      REQUIRE(voxel_of(stored) == voxel);
    }
  }
  for (const Eigen::Vector3s& point : points) {
    const std::optional<Eigen::Vector3s> stored = map.get_closest_neighbor(point, kVoxelSize);
    REQUIRE(stored.has_value());
    // half a quantum, except in the outermost half quantum of the voxel, which the clamp pulls one quantum in
    REQUIRE((*stored - point).cwiseAbs().maxCoeff() <= map.quantum);
  }
}

TEST_CASE("voxel_hash_map: a point on the quantum grid is stored exactly, lower faces included", "[voxel_hash_map]") {
  VoxelHashMap map(kVoxelSize, 100.0);
  // multiples of the quantum, so rounding has nothing to do; the second sits on its voxel's lower faces
  const std::vector<Eigen::Vector3s> points{{0.25, 0.5, 0.75}, {2.0, -3.0, 4.0}};
  map.add_points(points, Sophus::SE3s());

  for (const Eigen::Vector3s& point : points) {
    const std::optional<Eigen::Vector3s> stored = map.get_closest_neighbor(point, kVoxelSize);
    REQUIRE(stored.has_value());
    REQUIRE((*stored - point).norm() == 0.0);
  }
}
