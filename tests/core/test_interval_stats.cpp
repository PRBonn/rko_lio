#include "eigen_approx.hpp"

#include "rko_lio/core/util.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <numeric>
#include <vector>

using rko_lio::tests::TOL;
using rko_lio::tests::EXACT_TOL;
using rko_lio::core::IntervalStats;
using Catch::Matchers::WithinAbs;

TEST_CASE("IntervalStats: default-constructed state is zero", "[interval_stats]") {
  IntervalStats stats;
  REQUIRE(stats.imu_count == 0);
  REQUIRE(stats.angular_velocity_sum.isZero());
  REQUIRE(stats.body_acceleration_sum.isZero());
  REQUIRE(stats.imu_acceleration_sum.isZero());
  REQUIRE(stats.imu_accel_mag_mean == 0.0);
  REQUIRE(stats.welford_sum_of_squares == 0.0);
}

TEST_CASE("IntervalStats: single update", "[interval_stats]") {
  IntervalStats stats;
  const Eigen::Vector3s gyro(0.1, -0.2, 0.3);
  const Eigen::Vector3s accel(1.0, 2.0, 2.0);
  const Eigen::Vector3s compensated(0.0, 0.0, 0.0);
  stats.update(gyro, accel, compensated);

  REQUIRE(stats.imu_count == 1);
  REQUIRE(stats.angular_velocity_sum == gyro);
  REQUIRE(stats.imu_acceleration_sum == accel);
  REQUIRE(stats.body_acceleration_sum == compensated);
  REQUIRE_THAT(stats.imu_accel_mag_mean, WithinAbs(accel.norm(), EXACT_TOL));
  REQUIRE_THAT(stats.welford_sum_of_squares, WithinAbs(0.0, EXACT_TOL));
}

TEST_CASE("IntervalStats: constant accel magnitude over N updates -> variance = 0", "[interval_stats]") {
  IntervalStats stats;
  const Eigen::Vector3s gyro(0.0, 0.0, 0.0);
  const Eigen::Vector3s accel(0.0, 0.0, 9.81);
  const Eigen::Vector3s compensated(0.0, 0.0, 0.0);

  constexpr int N = 50;
  for (int i = 0; i < N; ++i) {
    stats.update(gyro, accel, compensated);
  }

  REQUIRE(stats.imu_count == N);
  REQUIRE_THAT(stats.imu_accel_mag_mean, WithinAbs(accel.norm(), TOL));
  REQUIRE_THAT(stats.welford_sum_of_squares, WithinAbs(0.0, TOL));
}

TEST_CASE("IntervalStats: variance matches naive formula", "[interval_stats]") {
  IntervalStats stats;
  // Vary only the z-axis to control the resulting scalar magnitudes.
  const std::vector<double> magnitudes = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  for (double m : magnitudes) {
    const Eigen::Vector3s accel(0.0, 0.0, m);
    stats.update(Eigen::Vector3s::Zero(), accel, Eigen::Vector3s::Zero());
  }

  const double mean = std::accumulate(magnitudes.begin(), magnitudes.end(), 0.0) / magnitudes.size();
  double naive_sum_sq = 0.0;
  for (double m : magnitudes) {
    naive_sum_sq += (m - mean) * (m - mean);
  }

  REQUIRE(stats.imu_count == static_cast<int>(magnitudes.size()));
  REQUIRE_THAT(stats.imu_accel_mag_mean, WithinAbs(mean, TOL));
  REQUIRE_THAT(stats.welford_sum_of_squares, WithinAbs(naive_sum_sq, TOL));

  // sample variance = sum_of_squares / (N-1)
  const double sample_var = stats.welford_sum_of_squares / (stats.imu_count - 1);
  const double naive_var = naive_sum_sq / (magnitudes.size() - 1);
  REQUIRE_THAT(sample_var, WithinAbs(naive_var, TOL));
}

TEST_CASE("IntervalStats: reset zeros every field", "[interval_stats]") {
  IntervalStats stats;
  stats.update(Eigen::Vector3s(1, 2, 3), Eigen::Vector3s(4, 5, 6), Eigen::Vector3s(7, 8, 9));
  stats.update(Eigen::Vector3s(1, 1, 1), Eigen::Vector3s(2, 2, 2), Eigen::Vector3s(3, 3, 3));
  REQUIRE(stats.imu_count == 2);

  stats.reset();
  REQUIRE(stats.imu_count == 0);
  REQUIRE(stats.angular_velocity_sum.isZero());
  REQUIRE(stats.body_acceleration_sum.isZero());
  REQUIRE(stats.imu_acceleration_sum.isZero());
  REQUIRE(stats.imu_accel_mag_mean == 0.0);
  REQUIRE(stats.welford_sum_of_squares == 0.0);
}

TEST_CASE("IntervalStats: sums accumulate across updates", "[interval_stats]") {
  IntervalStats stats;
  const Eigen::Vector3s g1(0.1, 0.2, 0.3);
  const Eigen::Vector3s g2(0.4, 0.5, 0.6);
  const Eigen::Vector3s a1(1.0, 0.0, 0.0);
  const Eigen::Vector3s a2(0.0, 1.0, 0.0);
  const Eigen::Vector3s c1(0.5, 0.0, 0.0);
  const Eigen::Vector3s c2(0.0, 0.5, 0.0);

  stats.update(g1, a1, c1);
  stats.update(g2, a2, c2);

  REQUIRE(stats.imu_count == 2);
  REQUIRE((stats.angular_velocity_sum - (g1 + g2)).norm() < EXACT_TOL);
  REQUIRE((stats.imu_acceleration_sum - (a1 + a2)).norm() < EXACT_TOL);
  REQUIRE((stats.body_acceleration_sum - (c1 + c2)).norm() < EXACT_TOL);
}
