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

add_library(rko_lio.core STATIC)
add_library(rko_lio::core ALIAS rko_lio.core)
compat_target_sources(
  rko_lio.core
  PRIVATE lio.cpp sparse_voxel_grid.cpp voxel_down_sample.cpp
          process_timestamps.cpp
  PUBLIC
  FILE_SET
  HEADERS
  FILES ../../rko_lio/core/lio.hpp
        ../../rko_lio/core/process_timestamps.hpp
        ../../rko_lio/core/profiler.hpp
        ../../rko_lio/core/sparse_voxel_grid.hpp
        ../../rko_lio/core/util.hpp
        ../../rko_lio/core/voxel_down_sample.hpp
  BASE_DIRS ../..)
target_link_libraries(
  rko_lio.core
  PRIVATE nlohmann_json::nlohmann_json TBB::tbb
  PUBLIC bonxai_core Sophus::Sophus Eigen3::Eigen)
target_compile_features(rko_lio.core PUBLIC cxx_std_20)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION
                                            VERSION_LESS 9)
  target_link_libraries(rko_lio.core PRIVATE stdc++fs)
endif()

# install is a problem because at least Bonxai is always fetch-contented, and
# that becomes a problem for the export target set. we'd have to manually export
# bonxai ourselves, which I would rather do as a patch or PR to upstream.
# therefore TODO
