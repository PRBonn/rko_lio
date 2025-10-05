include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++20" COMPILER_SUPPORTS_CXX20)
if(COMPILER_SUPPORTS_CXX20)
  set(NLOHMANN_JSON_VERSION "3.12.0")
else()
  set(NLOHMANN_JSON_VERSION "3.11.3")
endif()

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/archive/refs/tags/v${NLOHMANN_JSON_VERSION}.tar.gz
      ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(nlohmann_json)

mock_find_package_for_older_cmake(nlohmann_json)
