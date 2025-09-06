FetchContent_Declare(
  Bonxai
  GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
  GIT_TAG 47ffd0a2917c899f6199dfa71445481164298006 # apr 7 2025 master
  SOURCE_SUBDIR
  bonxai_core
  SYSTEM
  EXCLUDE_FROM_ALL
  OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(Bonxai)
