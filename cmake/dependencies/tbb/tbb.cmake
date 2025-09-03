# taken from kiss icp and simplified
# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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
option(BUILD_SHARED_LIBS OFF)
option(TBBMALLOC_BUILD OFF)
option(TBB_EXAMPLES OFF)
option(TBB_STRICT OFF)
option(TBB_TEST OFF)

if(RKO_LIO_ENABLE_FETCHCONTENT)
  include(FetchContent)
  FetchContent_Declare(
    tbb
    URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.1.0.tar.gz
        SYSTEM
        EXCLUDE_FROM_ALL
        OVERRIDE_FIND_PACKAGE)
  FetchContent_MakeAvailable(tbb)
else()
  add_subdirectory(
    ${CMAKE_CURRENT_LIST_DIR}/tbb
    ${CMAKE_BINARY_DIR}/tbb-oosbuild
    EXCLUDE_FROM_ALL
    SYSTEM)
endif()
