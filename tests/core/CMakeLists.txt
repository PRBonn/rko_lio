# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

add_executable(rko_lio_core_tests)

compat_target_sources(
  rko_lio_core_tests
  PRIVATE test_interval_stats.cpp
          test_voxel_down_sample.cpp
          test_process_timestamps.cpp
          test_preprocess_scan.cpp
          test_imu_integration.cpp
          test_register_scan.cpp
  PUBLIC
  FILE_SET
  HEADERS
  FILES eigen_approx.hpp synthetic_clouds.hpp
  BASE_DIRS .)

target_link_libraries(rko_lio_core_tests PRIVATE rko_lio::core
                                                 Catch2::Catch2WithMain)

target_compile_features(rko_lio_core_tests PRIVATE cxx_std_20)

catch_discover_tests(rko_lio_core_tests)
