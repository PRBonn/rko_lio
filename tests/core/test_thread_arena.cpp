#include "rko_lio/core/lio.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace rko_lio;

// max_num_threads used to be applied through a function-local static tbb::global_control, so only the
// first LIO built in a process ever set it, and it capped every other TBB user in that process too.
TEST_CASE("each LIO gets its own thread bound", "[threads]") {
  core::LIO::Config two;
  two.max_num_threads = 2;
  core::LIO::Config four;
  four.max_num_threads = 4;

  const core::LIO lio_two(two);
  const core::LIO lio_four(four);

  REQUIRE(lio_two.arena.max_concurrency() == 2);
  REQUIRE(lio_four.arena.max_concurrency() == 4);
}

TEST_CASE("max_num_threads of 0 leaves the arena at the TBB default", "[threads]") {
  core::LIO::Config config;
  config.max_num_threads = 0;
  const core::LIO lio(config);
  REQUIRE(lio.arena.max_concurrency() > 0);
}
