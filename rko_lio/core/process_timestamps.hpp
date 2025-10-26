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

/**
 * @file lio.hpp
 * Contains declarations for timestamp processing.
 */

#pragma once
#include "util.hpp"
#include <tuple>
#include <vector>

namespace rko_lio::core {
/**
 * Process raw timestamps and calculates absolute timestamps in seconds.
 *
 * The input timestamps can be either absolute or relative and may be in seconds or nanoseconds.
 * The function automatically detects whether the timestamps are in nanoseconds by checking the
 * duration spread and converts them to seconds accordingly.
 * Then the case of absolute or relative time is disambiguated based on how close (or away) the min or max times are to
 * the header time.
 *
 * @param raw_timestamps Vector of raw sensor timestamps as doubles, can be relative or absolute values.
 * @param header_stamp Reference absolute timestamp corresponding to the scan's header time.
 * @return A tuple containing:
 *   - The computed scan start time (absolute, Secondsd),
 *   - The computed scan end time (absolute, Secondsd),
 *   - A vector of all processed point timestamps converted to absolute seconds (TimestampVector).
 * @throws std::runtime_error If the timestamp format or values are unrecognized or unsupported.
 */
std::tuple<Secondsd, Secondsd, TimestampVector> process_timestamps(const std::vector<double>& raw_timestamps,
                                                                   const Secondsd& header_stamp);

} // namespace rko_lio::core
