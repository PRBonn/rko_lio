add_executable(rko_lio_core_tests)

compat_target_sources(
  rko_lio_core_tests
  PRIVATE test_interval_stats.cpp
          test_voxel_down_sample.cpp
          test_process_timestamps.cpp
          test_preprocess_scan.cpp
          test_thread_arena.cpp
          test_imu_integration.cpp
          test_register_scan.cpp
  PUBLIC
  FILE_SET
  HEADERS
  FILES eigen_approx.hpp synthetic_clouds.hpp
  BASE_DIRS .)

target_link_libraries(rko_lio_core_tests PRIVATE rko_lio::core
                                                 Catch2::Catch2WithMain)

target_compile_features(rko_lio_core_tests PRIVATE cxx_std_20)

catch_discover_tests(rko_lio_core_tests)
