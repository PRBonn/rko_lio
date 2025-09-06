FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz SYSTEM
      EXCLUDE_FROM_ALL OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(nlohmann_json)
