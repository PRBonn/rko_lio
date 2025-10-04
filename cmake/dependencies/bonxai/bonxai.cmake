FetchContent_Declare(
  Bonxai
  GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
  GIT_TAG 02d401b1ce38bce870c6704bcd4e35a56a641411 # sep 14 2025 master
  SOURCE_SUBDIR bonxai_core ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(Bonxai)

if(FETCHCONTENT_FULLY_DISCONNECTED)
  # the ros build farm uses this option to perform an isolated build. but as per
  # the author of FetchContent himself, this is an abuse of the flag. See
  # https://github.com/Homebrew/brew/pull/17075 and
  # https://gitlab.kitware.com/cmake/cmake/-/issues/25946. Nevertheless this is
  # a problem for us since Bonxai is not part of rosdistro yet. See here for
  # progress: https://github.com/facontidavide/Bonxai/issues/55. Since a user
  # can use this flag as valid behavior, the following is my best attempt at
  # catching the specific situation in the build farm. Hopefully soon enough, i
  # can remove this hack and stop vendoring bonxai code in my own repository.

  file(
    GLOB_RECURSE bonxai_source_files
    LIST_DIRECTORIES false
    "${bonxai_SOURCE_DIR}/*")

  list(LENGTH bonxai_source_files bonxai_source_count)

  if(bonxai_source_count EQUAL 0)
    message(
      WARNING
        "Bonxai source directory (${bonxai_SOURCE_DIR}) is empty and FETCHCONTENT_FULLY_DISCONNECTED is ON. This is likely unintended. Bonxai will be manually included for now as it is a required dependency."
    )
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bonxai_core)
  endif()

endif()
