option(BUILD_SHARED_LIBS OFF)
option(TBBMALLOC_BUILD OFF)
option(TBB_EXAMPLES OFF)
option(TBB_STRICT OFF)
option(TBB_TEST OFF)

FetchContent_Declare(
  TBB
  URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.2.0.tar.gz
      SYSTEM EXCLUDE_FROM_ALL OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(TBB)
