#include "rko_lio/core/preprocess_scan.hpp"
#include <catch2/catch_test_macros.hpp>

using rko_lio::core::LIO;
using rko_lio::core::preprocess_scan;
using rko_lio::core::Vector3sVector;

namespace {
LIO::Config default_config() {
  LIO::Config cfg;
  cfg.min_range = 0.5;
  cfg.max_range = 50.0;
  cfg.voxel_size = 0.5;
  cfg.double_downsample = true;
  cfg.deskew = true;
  return cfg;
}
} // namespace

TEST_CASE("preprocess_scan: clipping by min/max range", "[preprocess_scan]") {
  LIO::Config cfg = default_config();
  cfg.double_downsample = false;
  cfg.voxel_size = 0.05;

  Vector3sVector frame = {
      {0.1, 0.0, 0.0},
      {0.4, 0.0, 0.0},
      {1.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {49.0, 0.0, 0.0},
      {60.0, 0.0, 0.0},
      {100.0, 0.0, 0.0},
  };

  const auto result = preprocess_scan(frame, cfg);

  REQUIRE(result.filtered_frame.size() == 3);
  for (const auto& p : result.filtered_frame) {
    const double r = p.norm();
    REQUIRE(r > cfg.min_range);
    REQUIRE(r < cfg.max_range);
  }
}

TEST_CASE("preprocess_scan: double_downsample = false -> no map_frame", "[preprocess_scan]") {
  LIO::Config cfg = default_config();
  cfg.double_downsample = false;

  Vector3sVector frame;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      frame.emplace_back(2.0 + 0.05 * i, 2.0 + 0.05 * j, 1.0);
    }
  }

  const auto result = preprocess_scan(frame, cfg);
  REQUIRE(result.map_frame.empty());
  REQUIRE(result.keypoints.size() > 0);
  REQUIRE(result.filtered_frame.size() == frame.size());
}

TEST_CASE("preprocess_scan: double_downsample = true -> all three populated", "[preprocess_scan]") {
  LIO::Config cfg = default_config();
  cfg.double_downsample = true;

  Vector3sVector frame;
  for (int i = 0; i < 30; ++i) {
    for (int j = 0; j < 30; ++j) {
      frame.emplace_back(2.0 + 0.05 * i, 2.0 + 0.05 * j, 1.0);
    }
  }

  const auto result = preprocess_scan(frame, cfg);
  REQUIRE_FALSE(result.map_frame.empty());
  REQUIRE(result.map_frame.size() >= result.keypoints.size());
  REQUIRE(result.filtered_frame.size() == frame.size());
}

