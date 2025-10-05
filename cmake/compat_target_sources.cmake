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

# compat_target_sources macro: backward-compatible target_sources with FILE_SET
# for CMake < 3.23. Adds private sources normally, adds include directories
# fallback using BASE_DIRS
macro(compat_target_sources target)
  set(options)
  set(oneValueArgs BASE_DIRS)
  set(multiValueArgs
      PRIVATE
      PUBLIC
      FILE_SET
      HEADERS
      FILES)

  cmake_parse_arguments(
    CTS
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.23")
    target_sources(${target} ${ARGN})
  else()
    target_sources(${target} PRIVATE ${CTS_PRIVATE})
    target_include_directories(${target} PUBLIC "${CTS_BASE_DIRS}")
  endif()
endmacro()
