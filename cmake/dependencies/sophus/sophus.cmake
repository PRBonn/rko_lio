set(SOPHUS_USE_BASIC_LOGGING
    ON
    CACHE BOOL "Don't use fmt for Sophus library")
set(BUILD_SOPHUS_TESTS
    OFF
    CACHE BOOL "Don't build Sophus tests")
set(BUILD_SOPHUS_EXAMPLES
    OFF
    CACHE BOOL "Don't build Sophus Examples")
set(BUILD_PYTHON_BINDINGS
    OFF
    CACHE BOOL "Don't build Sophus Python Bindings")

FetchContent_Declare(
  Sophus URL https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.tar.gz
             ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(Sophus)

mock_find_package_for_older_cmake(Sophus)
