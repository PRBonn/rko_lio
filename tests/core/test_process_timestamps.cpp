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

#include "rko_lio/core/process_timestamps.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <stdexcept>

using rko_lio::core::process_timestamps;
using rko_lio::core::Secondsd;
using rko_lio::core::TimestampProcessingConfig;
using Catch::Matchers::WithinAbs;

namespace {
std::vector<double> linspace(double start, double end, size_t n) {
  std::vector<double> out(n);
  if (n == 1) {
    out[0] = start;
    return out;
  }
  const double step = (end - start) / static_cast<double>(n - 1);
  for (size_t i = 0; i < n; ++i) {
    out[i] = start + step * static_cast<double>(i);
  }
  return out;
}
} // namespace

TEST_CASE("process_timestamps: absolute seconds, header ~= min -> unchanged", "[process_timestamps]") {
  const auto raw = linspace(1234.0, 1234.1, 100);
  const Secondsd header{1234.0};
  const auto result = process_timestamps(raw, header, {});

  REQUIRE_THAT(result.min.count(), WithinAbs(1234.0, 1e-9));
  REQUIRE_THAT(result.max.count(), WithinAbs(1234.1, 1e-9));
  REQUIRE(result.times.size() == raw.size());
  REQUIRE_THAT(result.times.front().count(), WithinAbs(1234.0, 1e-9));
  REQUIRE_THAT(result.times.back().count(), WithinAbs(1234.1, 1e-9));
}

TEST_CASE("process_timestamps: relative seconds, min ~= 0 -> header offset added", "[process_timestamps]") {
  const auto raw = linspace(0.0, 0.1, 100);
  const Secondsd header{1234.5};
  const auto result = process_timestamps(raw, header, {});

  REQUIRE_THAT(result.min.count(), WithinAbs(1234.5, 1e-9));
  REQUIRE_THAT(result.max.count(), WithinAbs(1234.6, 1e-9));
  REQUIRE_THAT(result.times.front().count(), WithinAbs(1234.5, 1e-9));
  REQUIRE_THAT(result.times.back().count(), WithinAbs(1234.6, 1e-9));
}

TEST_CASE("process_timestamps: absolute nanoseconds -> heuristic infers 1e-9", "[process_timestamps]") {
  // Duration in ns is 1e8 > 100, which trips the nanosecond heuristic.
  const auto raw = linspace(1234.0e9, 1234.1e9, 100);
  const Secondsd header{1234.0};
  const auto result = process_timestamps(raw, header, {});

  REQUIRE_THAT(result.min.count(), WithinAbs(1234.0, 1e-3));
  REQUIRE_THAT(result.max.count(), WithinAbs(1234.1, 1e-3));
}

TEST_CASE("process_timestamps: relative nanoseconds", "[process_timestamps]") {
  const auto raw = linspace(0.0, 1.0e8, 100);
  const Secondsd header{1234.5};
  const auto result = process_timestamps(raw, header, {});

  REQUIRE_THAT(result.min.count(), WithinAbs(1234.5, 1e-3));
  REQUIRE_THAT(result.max.count(), WithinAbs(1234.6, 1e-3));
}

TEST_CASE("process_timestamps: force_absolute overrides heuristic", "[process_timestamps]") {
  // Stamps look relative (min ~= 0); force_absolute keeps them as-is.
  const auto raw = linspace(0.0, 0.1, 100);
  const Secondsd header{1234.5};
  TimestampProcessingConfig cfg;
  cfg.force_absolute = true;
  const auto result = process_timestamps(raw, header, cfg);

  REQUIRE_THAT(result.min.count(), WithinAbs(0.0, 1e-9));
  REQUIRE_THAT(result.max.count(), WithinAbs(0.1, 1e-9));
}

TEST_CASE("process_timestamps: force_relative overrides heuristic", "[process_timestamps]") {
  // Stamps that match neither absolute nor relative heuristic; force_relative routes them to relative.
  TimestampProcessingConfig cfg;
  cfg.force_relative = true;
  const auto raw = linspace(50.0, 50.1, 100);
  const Secondsd header{1000.0};
  const auto result = process_timestamps(raw, header, cfg);

  REQUIRE_THAT(result.min.count(), WithinAbs(1050.0, 1e-9));
  REQUIRE_THAT(result.max.count(), WithinAbs(1050.1, 1e-9));
}

TEST_CASE("process_timestamps: custom multiplier_to_seconds is honored", "[process_timestamps]") {
  const auto raw = linspace(0.0, 1.0e5, 100);
  const Secondsd header{1234.5};
  TimestampProcessingConfig cfg;
  cfg.multiplier_to_seconds = 1e-6;
  const auto result = process_timestamps(raw, header, cfg);

  REQUIRE_THAT(result.min.count(), WithinAbs(1234.5, 1e-3));
  REQUIRE_THAT(result.max.count(), WithinAbs(1234.6, 1e-3));
}

TEST_CASE("process_timestamps: ambiguous case throws", "[process_timestamps]") {
  const auto raw = linspace(50.0, 50.1, 100);
  const Secondsd header{1000.0};
  REQUIRE_THROWS_AS(process_timestamps(raw, header, {}), std::runtime_error);
}
