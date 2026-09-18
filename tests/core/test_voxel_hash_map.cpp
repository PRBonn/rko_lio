#include "rko_lio/core/voxel_hash_map.hpp"
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <vector>

using Catch::Generators::from_range;
using rko_lio::core::point_to_voxel;
using rko_lio::core::Scalar;
using rko_lio::core::VoxelHashMap;

namespace {
// sizes Scalar represents exactly. At other sizes point_to_voxel's reciprocal multiply and the voxel centre disagree
// at the faces, and a decoded point can bin one voxel down
constexpr std::array VOXEL_SIZES{0.25, 0.5, 0.75, 1.0};
} // namespace

TEST_CASE("voxel_hash_map: a stored point reads back inside its own voxel", "[voxel_hash_map]") {
  const auto voxel_size = static_cast<Scalar>(GENERATE(from_range(VOXEL_SIZES)));
  VoxelHashMap map(voxel_size, 100.0);

  std::vector<Eigen::Vector3s> points;
  for (int index = -3; index < 3; ++index) {
    const Scalar corner = static_cast<Scalar>(index) * voxel_size;
    const Scalar below_upper_face = corner + voxel_size - (static_cast<Scalar>(0.25) * map.quantum);
    points.emplace_back(below_upper_face, corner + (voxel_size / 2), below_upper_face);
  }
  map.add_points(points, Sophus::SE3s());
  REQUIRE(map.voxels.size() == points.size());

  for (const Eigen::Vector3s& stored : map.points()) {
    REQUIRE(map.voxels.find(point_to_voxel(stored, 1 / voxel_size)) != map.voxels.end());
  }
  for (const Eigen::Vector3s& point : points) {
    const std::optional<Eigen::Vector3s> stored = map.get_closest_neighbor(point, voxel_size);
    REQUIRE(stored.has_value());
    // a whole quantum, not half: these points sit in the band the clamp pulls one quantum in
    REQUIRE((*stored - point).cwiseAbs().maxCoeff() <= map.quantum);
  }
}

TEST_CASE("voxel_hash_map: a point on the quantum grid is stored exactly, lower faces included", "[voxel_hash_map]") {
  const auto voxel_size = static_cast<Scalar>(GENERATE(from_range(VOXEL_SIZES)));
  VoxelHashMap map(voxel_size, 100.0);
  const std::vector<Eigen::Vector3s> points{
      {voxel_size / 4, voxel_size / 2, 3 * voxel_size / 4},
      {2 * voxel_size, -3 * voxel_size, 4 * voxel_size},
  };
  map.add_points(points, Sophus::SE3s());

  for (const Eigen::Vector3s& point : points) {
    const std::optional<Eigen::Vector3s> stored = map.get_closest_neighbor(point, voxel_size);
    REQUIRE(stored.has_value());
    REQUIRE((*stored - point).norm() == 0.0);
  }
}
