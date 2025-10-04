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

macro(mock_find_package_for_older_cmake PACKAGE_NAME)
  # Only if CMake < 3.24 (no OVERRIDE_FIND_PACKAGE)
  if(NOT CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    set(MOCK_CONFIG_DIR "${CMAKE_BINARY_DIR}/cmake-mock-configs")
    if(NOT EXISTS "${MOCK_CONFIG_DIR}")
      file(MAKE_DIRECTORY "${MOCK_CONFIG_DIR}")
    endif()

    list(APPEND CMAKE_PREFIX_PATH "${MOCK_CONFIG_DIR}")
    list(REMOVE_DUPLICATES CMAKE_PREFIX_PATH)
    set(CMAKE_PREFIX_PATH
        "${CMAKE_PREFIX_PATH}"
        CACHE STRING "Mock config path" FORCE)

    set(MOCK_CONFIG_FILE "${MOCK_CONFIG_DIR}/${PACKAGE_NAME}Config.cmake")

    if(NOT EXISTS "${MOCK_CONFIG_FILE}")
      file(WRITE "${MOCK_CONFIG_FILE}" "set(${PACKAGE_NAME}_FOUND TRUE)\n")
    endif()
  endif()
endmacro()
