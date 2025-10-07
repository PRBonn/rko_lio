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

include(FetchContent)

option(RKO_LIO_FETCH_CONTENT_DEPS
       "Fetch dependencies via FetchContent instead of using find_package" OFF)

set(RKO_LIO_FETCHCONTENT_COMMON_FLAGS)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
  list(APPEND RKO_LIO_FETCHCONTENT_COMMON_FLAGS OVERRIDE_FIND_PACKAGE)
endif()
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.25")
  list(APPEND RKO_LIO_FETCHCONTENT_COMMON_FLAGS SYSTEM)
endif()
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.28")
  list(APPEND RKO_LIO_FETCHCONTENT_COMMON_FLAGS EXCLUDE_FROM_ALL)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/mock_find_package.cmake)

# Bonxai is a special case as upstream releases no system version
include(${CMAKE_CURRENT_LIST_DIR}/dependencies/bonxai/bonxai.cmake)

if(RKO_LIO_FETCH_CONTENT_DEPS)
  include(${CMAKE_CURRENT_LIST_DIR}/dependencies/eigen/eigen.cmake)
  include(${CMAKE_CURRENT_LIST_DIR}/dependencies/sophus/sophus.cmake)
  include(${CMAKE_CURRENT_LIST_DIR}/dependencies/tbb/tbb.cmake)
  include(${CMAKE_CURRENT_LIST_DIR}/dependencies/json/nlohmann_json.cmake)
endif()
