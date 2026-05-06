/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
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

#include "process_timestamps.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using rko_lio::core::Nsec;
using rko_lio::core::TimestampProcessingConfig;
using rko_lio::core::Timestamps;
using rko_lio::core::TimestampVector;
using namespace std::chrono_literals;

inline Nsec raw_to_ns(const double ts, const double multiplier_to_ns, const bool source_is_ns) {
  if (source_is_ns) {
    return Nsec(static_cast<int64_t>(ts));
  }
  return Nsec(static_cast<int64_t>(std::llround(ts * multiplier_to_ns)));
}

// The timestamps are either in seconds or in nanoseconds, otherwise the user needs to specify the multiplier
Timestamps timestamps_from_raw(const std::vector<double>& raw_timestamps, const double multiplier_to_seconds) {
  const auto& [min_it, max_it] = std::minmax_element(raw_timestamps.begin(), raw_timestamps.end());
  const double min_stamp = *min_it;
  const double max_stamp = *max_it;

  bool source_is_ns = false;
  double multiplier_to_ns = 0.0;
  if (multiplier_to_seconds < 1e-12) {
    const double scan_duration = std::abs(max_stamp - min_stamp);
    // 100 seconds is far more than the duration of any normal scan
    source_is_ns = (scan_duration > 100.0);
    multiplier_to_ns = source_is_ns ? 1.0 : 1e9;
  } else {
    multiplier_to_ns = multiplier_to_seconds * 1e9;
  }

  TimestampVector times_ns(raw_timestamps.size());
  std::transform(raw_timestamps.cbegin(), raw_timestamps.cend(), times_ns.begin(),
                 [multiplier_to_ns, source_is_ns](const double ts) {
                   return raw_to_ns(ts, multiplier_to_ns, source_is_ns);
                 });
  return {.min = raw_to_ns(min_stamp, multiplier_to_ns, source_is_ns),
          .max = raw_to_ns(max_stamp, multiplier_to_ns, source_is_ns),
          .times = times_ns};
}
} // namespace

namespace rko_lio::core {
Timestamps process_timestamps(const std::vector<double>& raw_timestamps,
                              const Nsec header_stamp,
                              const TimestampProcessingConfig& config) {
  Timestamps timestamps = timestamps_from_raw(raw_timestamps, config.multiplier_to_seconds);

  const bool absolute_stamps = config.force_absolute || (std::chrono::abs(header_stamp - timestamps.min) < 1ms) ||
                               (std::chrono::abs(header_stamp - timestamps.max) < 10ms);

  if (absolute_stamps) {
    return timestamps;
  }

  const bool relative_stamps =
      config.force_relative || (std::chrono::abs(timestamps.min) < 1ms) || (std::chrono::abs(timestamps.max) < 10ms);

  if (relative_stamps) {
    std::transform(timestamps.times.cbegin(), timestamps.times.cend(), timestamps.times.begin(),
                   [header_stamp](const Nsec ts) { return ts + header_stamp; });
    timestamps.min += header_stamp;
    timestamps.max += header_stamp;
    return timestamps;
  }

  // Error-out for unique/unsupported cases
  std::cout << std::setprecision(18);
  std::cout << "is_absolute: " << absolute_stamps << "\n";
  std::cout << "is_relative: " << relative_stamps << "\n";
  std::cout << "point times min (ns): " << timestamps.min.count() << "\n";
  std::cout << "point times max (ns): " << timestamps.max.count() << "\n";
  std::cout << "header_stamp (ns): " << header_stamp.count() << "\n";
  std::cout << "TimestampProcessingConfig:\n"
            << "  multiplier_to_seconds: " << config.multiplier_to_seconds << "\n"
            << "  force_absolute: " << std::boolalpha << config.force_absolute << "\n"
            << "  force_relative: " << std::boolalpha << config.force_relative << "\n"
            << "  start and end difference thresholds to header time are 1 and 10 milliseconds. Please force a case if "
               "those thresholds don't suit your sensor.\n"
            << std::noboolalpha;
  throw std::runtime_error(
      "TimestampProcessingConfig does not cover this particular case of data. Please investigate, modify "
      "the config, or open an issue.");
}
} // namespace rko_lio::core
