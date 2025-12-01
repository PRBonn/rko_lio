set(EIGEN_BUILD_DOC
    OFF
    CACHE BOOL "Don't build Eigen docs")
set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "Don't build Eigen tests")
set(BUILD_TESTING
    OFF
    CACHE BOOL "Don't build Eigen tests but without the EIGEN prefix")
set(EIGEN_BUILD_PKGCONFIG
    OFF
    CACHE BOOL "Don't build Eigen pkg-config")
set(EIGEN_BUILD_BLAS
    OFF
    CACHE BOOL "Don't build blas module")
set(EIGEN_BUILD_LAPACK
    OFF
    CACHE BOOL "Don't build lapack module")

FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
      ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(Eigen3)

mock_find_package_for_older_cmake(Eigen3)
