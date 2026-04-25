// MIT License
//
// Copyright (c) 2025 Meher V.R. Malladi.
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

#include "rko_lio/core/preprocess_scan.hpp"
#include <catch2/catch_test_macros.hpp>

using rko_lio::core::LIO;
using rko_lio::core::preprocess_scan;
using rko_lio::core::Vector3dVector;

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

  Vector3dVector frame = {
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

  Vector3dVector frame;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      frame.emplace_back(2.0 + 0.05 * i, 2.0 + 0.05 * j, 1.0);
    }
  }

  const auto result = preprocess_scan(frame, cfg);
  REQUIRE_FALSE(result.map_frame.has_value());
  REQUIRE(result.keypoints.size() > 0);
  REQUIRE(result.filtered_frame.size() == frame.size());
  REQUIRE(result.map_update_frame().size() == result.keypoints.size());
}

TEST_CASE("preprocess_scan: double_downsample = true -> all three populated", "[preprocess_scan]") {
  LIO::Config cfg = default_config();
  cfg.double_downsample = true;

  Vector3dVector frame;
  for (int i = 0; i < 30; ++i) {
    for (int j = 0; j < 30; ++j) {
      frame.emplace_back(2.0 + 0.05 * i, 2.0 + 0.05 * j, 1.0);
    }
  }

  const auto result = preprocess_scan(frame, cfg);
  REQUIRE(result.map_frame.has_value());
  REQUIRE(result.map_frame->size() >= result.keypoints.size());
  REQUIRE(result.filtered_frame.size() == frame.size());
  REQUIRE(result.map_update_frame().size() == result.map_frame->size());
}

