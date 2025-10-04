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
  if(NOT ARGC GREATER 0)
    message(FATAL_ERROR "compat_target_sources requires at least 1 argument")
  endif()
  set(all_args ${ARGV})
  list(REMOVE_AT all_args 0)
  set(target ${ARGV0})
  set(args ${all_args})

  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.23")
    target_sources(${target} ${args})
  else()
    set(private_sources)
    set(base_dir "")
    set(in_private OFF)
    set(in_public OFF)
    foreach(arg IN LISTS args)
      if(arg STREQUAL "PRIVATE")
        set(in_private ON)
        set(in_public OFF)
        continue()
      elseif(arg STREQUAL "PUBLIC")
        set(in_private OFF)
        set(in_public ON)
        continue()
      elseif(arg STREQUAL "BASE_DIRS")
        set(in_private OFF)
        set(in_public OFF)
        continue()
      endif()

      if(in_private)
        list(APPEND private_sources ${arg})
      elseif(
        NOT in_private
        AND NOT in_public
        AND NOT base_dir)
        set(base_dir ${arg})
      endif()
    endforeach()

    target_sources(${target} PRIVATE ${private_sources})
    target_include_directories(${target} PUBLIC "${base_dir}")
  endif()
endmacro()
