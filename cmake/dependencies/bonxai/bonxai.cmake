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

FetchContent_Declare(
  Bonxai
  GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
  GIT_TAG 02d401b1ce38bce870c6704bcd4e35a56a641411 # sep 14 2025 master
  SOURCE_SUBDIR bonxai_core ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})

if(NOT
   (CMAKE_EXPORT_NO_PACKAGE_REGISTRY STREQUAL "ON"
    AND CMAKE_FIND_USE_PACKAGE_REGISTRY STREQUAL "OFF"
    AND CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY STREQUAL "ON"))
  # This branch is usual behaviour. The else branch is relevant for network
  # isolated builds
  FetchContent_MakeAvailable(Bonxai)
else()
  message(
    WARNING
      "You're probably doing a network-isolated build. But Bonxai is a required dependency which should be fetched. It's being manually vendored in the repo for now for this purpose."
  )
  if(NOT FETCHCONTENT_FULLY_DISCONNECTED)
    # noble isolated builds have this as ON. the purpose of this if is to just
    # emit a warning in case a user unintentionally ends up in this branch. as
    # per the author of FetchContent himself, the build farm use is an abuse of
    # the flag. See https://github.com/Homebrew/brew/pull/17075 and
    # https://gitlab.kitware.com/cmake/cmake/-/issues/25946. Nevertheless this
    # is a problem for us since Bonxai is not part of rosdistro yet. See here
    # for progress: https://github.com/facontidavide/Bonxai/issues/55. Hopefully
    # soon enough, i can remove this hack and stop vendoring bonxai code in my
    # own repository.
    message(
      WARNING
        "It looks like an isolated build is happening, but FETCHCONTENT_FULLY_DISCONNECTED is not ON."
    )
  endif()
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bonxai_core)
endif()

mock_find_package_for_older_cmake(Bonxai)
