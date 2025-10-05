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

if(CMAKE_VERSION VERSION_LESS "3.24")
  set(SOPHUS_VERSION "1.22.10")
else()
  set(SOPHUS_VERSION "1.24.6")
endif()

FetchContent_Declare(
  Sophus
  URL https://github.com/strasdat/Sophus/archive/refs/tags/${SOPHUS_VERSION}.tar.gz
      ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(Sophus)

mock_find_package_for_older_cmake(Sophus)
