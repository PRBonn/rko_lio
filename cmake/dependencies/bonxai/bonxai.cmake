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

option(RKO_LIO_FORCE_INCLUDE_BONXAI
       "Force add_subdirectory of Bonxai vendored code if not fetching it" ON)

if(RKO_LIO_FETCHCONTENT_DEPS)
  FetchContent_MakeAvailable(Bonxai)
elseif(RKO_LIO_FORCE_INCLUDE_BONXAI)
  # ROS build farms are either network isolated (ubuntu builds) or do not have
  # git available to clone packages during cmake configure (rhel builds). This
  # is a problem for us since Bonxai is not part of rosdistro yet. See here for
  # progress: https://github.com/facontidavide/Bonxai/issues/55. I earlier had
  # some conditions based on cmake flags, but thats brittle. This way, a user
  # can just disable this behaviour on purpose. Hopefully soon enough, i can
  # remove this hack and stop vendoring bonxai code in my own repository. And no
  # i do not want to use submodules
  message(
    WARNING
      "Including Bonxai source code vendored in rko_lio. But Bonxai is a required dependency which should be fetched. This is a temporary solution until upstream is available via system vendors."
  )
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bonxai_core)
endif()

mock_find_package_for_older_cmake(Bonxai)
