option(SPDLOG_BUILD_PIC "Build position independent code (-fPIC)" ON)

FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/archive/refs/tags/v1.17.0.tar.gz
                                ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(spdlog)

mock_find_package_for_older_cmake(spdlog)
