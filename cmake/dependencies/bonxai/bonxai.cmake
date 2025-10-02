FetchContent_Declare(
  Bonxai
  GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
  GIT_TAG 02d401b1ce38bce870c6704bcd4e35a56a641411 # sep 14 2025 master
  SOURCE_SUBDIR
  bonxai_core
  SYSTEM
  EXCLUDE_FROM_ALL
  OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(Bonxai)
