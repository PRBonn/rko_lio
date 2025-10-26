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
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using rko_lio::core::Secondsd;
using rko_lio::core::TimestampVector;

// Returns the number of integer digits in a value.
inline int number_of_digits_integer_part(const double value) {
  const double abs_val = std::abs(value);
  const auto int_part = static_cast<uint64_t>(std::round(abs_val));
  return int_part > 0 ? static_cast<int>(std::floor(std::log10(int_part) + 1)) : 1;
}

// The timestamps are either in seconds or in nanoseconds, we handle no other case for now
std::tuple<Secondsd, Secondsd, TimestampVector> timestamps_in_sec_from_raw(const std::vector<double>& raw_timestamps) {
  const auto& [min_it, max_it] = std::minmax_element(raw_timestamps.begin(), raw_timestamps.end());
  const double min_stamp = *min_it;
  const double max_stamp = *max_it;
  const double scan_duration = std::abs(max_stamp - min_stamp);

  const bool is_nanoseconds = (scan_duration > 100.0);
  // 100 seconds is far more than the duration of any normal scan
  // but note that this was necessary because a hesai sensor spit out a 20 sec difference in a real scan
  // I'll welcome improvements to logic here to handle edge cases like this

  TimestampVector timestamps_in_seconds(raw_timestamps.size());
  std::transform(raw_timestamps.cbegin(), raw_timestamps.cend(), timestamps_in_seconds.begin(),
                 [is_nanoseconds](const double ts) { return Secondsd(is_nanoseconds ? ts * 1e-9 : ts); });
  if (is_nanoseconds) {
    return {Secondsd(min_stamp * 1e-9), Secondsd(max_stamp * 1e-9), timestamps_in_seconds};
  } else {
    return {Secondsd(min_stamp), Secondsd(max_stamp), timestamps_in_seconds};
  }
}
} // namespace

namespace rko_lio::core {
std::tuple<Secondsd, Secondsd, TimestampVector> process_timestamps(const std::vector<double>& raw_timestamps,
                                                                   const Secondsd& header_stamp) {
  constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);

  const auto& [min_ts, max_ts, raw_timestamps_in_sec] = timestamps_in_sec_from_raw(raw_timestamps);
  TimestampVector timestamps = raw_timestamps_in_sec;
  const Secondsd scan_duration = std::chrono::abs(max_ts - min_ts);

  Secondsd begin_stamp;
  Secondsd end_stamp;

  // Case: absolute timestamps
  // min or max is within 1ms to stamp time. If the timestamps are not absolute, I don't know what they are.
  if (std::chrono::abs(header_stamp - min_ts) < std::chrono::milliseconds(1) ||
      std::chrono::abs(header_stamp - max_ts) < std::chrono::milliseconds(1)) {
    begin_stamp = min_ts;
    end_stamp = max_ts;
    return {begin_stamp, end_stamp, std::move(timestamps)};
  }

  // - Case 1: timestamps are relative to scan start
  // - Case 2: timestamps are relative to scan end
  //   the limits i've set empirically based on all the cases encountered till now and what felt reasonable
  bool relative_stamps = (min_ts >= Secondsd(0) && std::chrono::abs(min_ts) < std::chrono::milliseconds(4)) ||
                         (max_ts <= Secondsd(0) && std::chrono::abs(max_ts) < std::chrono::milliseconds(4));

  // here are weirdo cases i've seen which i think should be kept separate so its always in mind in case they cause
  // problems in the future
  // - Avia Livox on HeLiPR - min timestamps are oten greater than the header stamp by a significant amount
  // - VLP-16 on Leg-KILO: relative to the end, but some points are ahead of the header stamp
  relative_stamps = relative_stamps ||
                    (min_ts >= Secondsd(0) && std::chrono::abs(min_ts) < std::chrono::milliseconds(20)) ||
                    (min_ts < Secondsd(0) && max_ts > Secondsd(0) && max_ts < std::chrono::milliseconds(10));

  if (relative_stamps) {
    std::transform(timestamps.begin(), timestamps.end(), timestamps.begin(),
                   [&header_stamp](const Secondsd& ts) { return ts + header_stamp; });
    begin_stamp = min_ts + header_stamp;
    end_stamp = max_ts + header_stamp;
    return {begin_stamp, end_stamp, std::move(timestamps)};
  }

  // Error-out for unique/unsupported cases
  std::cout << std::setprecision(18);
  std::cout << "is_relative: " << relative_stamps << "\n";
  std::cout << "point min_time: " << min_ts.count() << "\n";
  std::cout << "point max_time: " << max_ts.count() << "\n";
  std::cout << "header_sec: " << header_stamp.count() << "\n";
  std::cout << "header_sec + min_time: " << (header_stamp + min_ts).count() << "\n";
  std::cout << "header_sec + max_time: " << (header_stamp + max_ts).count() << "\n";
  throw std::runtime_error(
      "Can not handle timestamp conversion. Some unique case encountered. Please report an issue with this data.");

  // NOTE: one could assume that once a case has been detected for one scan, it should probably stay the same going
  // forward for the current run at least. In case someone needs more performance, this is an easy improvement
}
} // namespace rko_lio::core
