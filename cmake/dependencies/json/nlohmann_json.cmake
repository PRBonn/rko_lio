FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz
      ${RKO_LIO_FETCHCONTENT_COMMON_FLAGS})
FetchContent_MakeAvailable(nlohmann_json)
