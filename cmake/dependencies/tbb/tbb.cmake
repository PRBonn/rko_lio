option(BUILD_SHARED_LIBS OFF)
option(TBBMALLOC_BUILD OFF)
option(TBB_EXAMPLES OFF)
option(TBB_STRICT OFF)
option(TBB_TEST OFF)

FetchContent_Declare(
  TBB
  URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.2.0.tar.gz
      ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(TBB)

mock_find_package_for_older_cmake(TBB)
