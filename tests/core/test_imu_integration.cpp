// MIT License
//
// Copyright (c) 2025 Meher V.R. Malladi.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "eigen_approx.hpp"
#include "rko_lio/core/lio.hpp"
#include "rko_lio/core/util.hpp"
#include "synthetic_clouds.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using rko_lio::core::GRAVITY_MAG;
using rko_lio::core::ImuControl;
using rko_lio::core::LIO;
using rko_lio::core::Secondsd;
using rko_lio::core::TimestampVector;
using rko_lio::core::Vector3dVector;
using rko_lio::tests::approx_equal;
using rko_lio::tests::make_hollow_cube;
using Catch::Matchers::WithinAbs;

namespace {
LIO::Config default_config() {
  LIO::Config cfg;
  cfg.min_range = 0.5;
  cfg.max_range = 50.0;
  cfg.voxel_size = 0.5;
  cfg.deskew = true;
  cfg.double_downsample = true;
  return cfg;
}
} // namespace

TEST_CASE("IMU before first LiDAR is silently dropped", "[imu_integration]") {
  LIO lio(default_config());
  for (int i = 0; i < 5; ++i) {
    ImuControl m;
    m.time = Secondsd{0.01 * (i + 1)};
    m.acceleration = {0.0, 0.0, GRAVITY_MAG};
    m.angular_velocity = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(m);
  }
  REQUIRE(lio.interval_stats.imu_count == 0);
}

TEST_CASE("Static IMU at rest: gravity is compensated", "[imu_integration]") {
  LIO lio(default_config());
  // Bootstrap as if a first lidar scan had been registered.
  lio.lidar_state.time = Secondsd{1.0};

  constexpr int N = 50;
  for (int i = 0; i < N; ++i) {
    ImuControl m;
    m.time = Secondsd{1.0 + 0.01 * (i + 1)};
    m.acceleration = {0.0, 0.0, GRAVITY_MAG};
    m.angular_velocity = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(m);
  }

  REQUIRE(lio.interval_stats.imu_count == N);
  REQUIRE(lio.interval_stats.body_acceleration_sum.norm() < 1e-9);
  REQUIRE_THAT(lio.interval_stats.imu_acceleration_sum.z(), WithinAbs(N * GRAVITY_MAG, 1e-6));
}

TEST_CASE("Pure rotation: gyro integration matches exp(omega*T)", "[imu_integration]") {
  LIO lio(default_config());
  lio.lidar_state.time = Secondsd{1.0};

  const Eigen::Vector3d omega(0.0, 0.0, 0.5);
  constexpr int N = 100;
  constexpr double dt = 0.01;
  const double T = N * dt;

  for (int i = 0; i < N; ++i) {
    ImuControl m;
    m.time = Secondsd{1.0 + dt * (i + 1)};
    m.angular_velocity = omega;
    m.acceleration = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(m);
  }

  const Sophus::SO3d expected = Sophus::SO3d::exp(omega * T);
  REQUIRE(approx_equal(lio.imu_state.pose.so3(), expected, 1e-9));
  REQUIRE_THAT(lio.imu_state.time.count(), WithinAbs(1.0 + T, 1e-12));
}

TEST_CASE("Gyro bias is subtracted before integration", "[imu_integration]") {
  LIO lio(default_config());
  lio.lidar_state.time = Secondsd{1.0};
  lio.imu_bias.gyroscope = {0.1, -0.2, 0.05};

  const Eigen::Vector3d measured(0.5, 0.3, 0.1);
  constexpr int N = 50;
  for (int i = 0; i < N; ++i) {
    ImuControl m;
    m.time = Secondsd{1.0 + 0.01 * (i + 1)};
    m.angular_velocity = measured;
    m.acceleration = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(m);
  }

  const Eigen::Vector3d expected = measured - lio.imu_bias.gyroscope;
  const Eigen::Vector3d avg = lio.interval_stats.angular_velocity_sum / lio.interval_stats.imu_count;
  REQUIRE(approx_equal(avg, expected, 1e-12));
}

TEST_CASE("Extrinsic overload: identity extrinsic delegates to base-frame", "[imu_integration]") {
  LIO lio(default_config());
  lio.lidar_state.time = Secondsd{1.0};

  const Sophus::SE3d identity_extrinsic;
  const Eigen::Vector3d omega(0.1, 0.2, 0.3);

  constexpr int N = 10;
  for (int i = 0; i < N; ++i) {
    ImuControl m;
    m.time = Secondsd{1.0 + 0.01 * (i + 1)};
    m.angular_velocity = omega;
    m.acceleration = {0.0, 0.0, GRAVITY_MAG};
    lio.add_imu_measurement(identity_extrinsic, m);
  }

  // Identity-extrinsic short-circuits straight into the base-frame overload,
  // bypassing the extrinsic-overload's first-sample skip.
  REQUIRE(lio.interval_stats.imu_count == N);
  const Eigen::Vector3d avg_gyro = lio.interval_stats.angular_velocity_sum / lio.interval_stats.imu_count;
  REQUIRE(approx_equal(avg_gyro, omega, 1e-12));
}

TEST_CASE("Extrinsic overload: pure-rotation extrinsic transforms omega", "[imu_integration]") {
  LIO lio(default_config());
  lio.lidar_state.time = Secondsd{1.0};

  const Sophus::SO3d R = Sophus::SO3d::rotX(M_PI / 2.0);
  const Sophus::SE3d extrinsic_imu2base(R, Eigen::Vector3d::Zero());

  const Eigen::Vector3d omega_imu(0.0, 0.0, 1.0);
  const Eigen::Vector3d expected_base = R * omega_imu;

  // First extrinsic call is dropped (primes _last_real_imu_time); feed N+1 to get N integrated.
  constexpr int N = 20;
  for (int i = 0; i < N + 1; ++i) {
    ImuControl m;
    m.time = Secondsd{1.0 + 0.01 * (i + 1)};
    m.angular_velocity = omega_imu;
    m.acceleration = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(extrinsic_imu2base, m);
  }

  REQUIRE(lio.interval_stats.imu_count == N);
  const Eigen::Vector3d avg_gyro = lio.interval_stats.angular_velocity_sum / lio.interval_stats.imu_count;
  REQUIRE(approx_equal(avg_gyro, expected_base, 1e-9));
}

// TODO: lever-arm centripetal correction (omega x (omega x r)) for translational extrinsic.

TEST_CASE("Static IMU at rest: imu_state translation stays zero", "[imu_integration]") {
  LIO lio(default_config());
  // Bootstrap as if a first lidar scan had been registered. The first add_imu_measurement
  // hits the lazy-init branch and anchors imu_state.pose = identity, velocity = 0.
  lio.lidar_state.time = Secondsd{1.0};

  constexpr int N = 50;
  for (int i = 0; i < N; ++i) {
    ImuControl m;
    m.time = Secondsd{1.0 + 0.01 * (i + 1)};
    m.acceleration = {0.0, 0.0, GRAVITY_MAG};
    m.angular_velocity = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(m);
  }

  REQUIRE(lio.interval_stats.imu_count == N);
  REQUIRE(lio.imu_state.pose.translation().norm() < 1e-9);
  REQUIRE(lio.imu_state.velocity.norm() < 1e-9);
}

TEST_CASE("Constant body acceleration: position grows as 0.5 * a * t^2", "[imu_integration]") {
  LIO lio(default_config());
  // Bootstrap as if a first lidar scan had been registered with identity pose.
  lio.lidar_state.time = Secondsd{1.0};

  constexpr double a_x = 0.5; // m/s^2 in body x
  constexpr int N = 100;
  constexpr double dt = 0.01;
  const double T = N * dt;

  for (int i = 0; i < N; ++i) {
    ImuControl m;
    m.time = Secondsd{1.0 + dt * (i + 1)};
    // Raw IMU acceleration: body accel + gravity (so compensated_accel comes out (a_x, 0, 0)).
    m.acceleration = {a_x, 0.0, GRAVITY_MAG};
    m.angular_velocity = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(m);
  }

  // Tolerance reflects Euler-step truncation error: each step's position picks up
  // a_x * dt^2 / 2 and the integrator effectively uses pre-step velocity, so the
  // accumulated error after N steps is ~N * dt^2 * a_x / 2 = T * dt * a_x / 2.
  const double pos_tol = N * dt * dt * a_x;
  const double vel_tol = 1e-9;
  REQUIRE(lio.interval_stats.imu_count == N);
  REQUIRE_THAT(lio.imu_state.pose.translation().x(), WithinAbs(0.5 * a_x * T * T, pos_tol));
  REQUIRE(std::abs(lio.imu_state.pose.translation().y()) < 1e-9);
  REQUIRE(std::abs(lio.imu_state.pose.translation().z()) < 1e-9);
  REQUIRE_THAT(lio.imu_state.velocity.x(), WithinAbs(a_x * T, vel_tol));
  REQUIRE(std::abs(lio.imu_state.velocity.y()) < 1e-9);
  REQUIRE(std::abs(lio.imu_state.velocity.z()) < 1e-9);
}

TEST_CASE("register_scan resets imu_state to optimized pose", "[imu_integration]") {
  // Mirrors the "Identity registration" test in test_register_scan.cpp: register the
  // same hollow-cube cloud twice with a static IMU between them, then assert that
  // imu_state was re-anchored to lidar_state at scan end.
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  constexpr double FIRST_SCAN_END = 0.1;
  constexpr double DT = 1.0;
  constexpr double SECOND_SCAN_END = FIRST_SCAN_END + DT;

  // Build per-point timestamps for the first scan (linspace from 0 to FIRST_SCAN_END).
  TimestampVector ts1;
  ts1.reserve(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    const double frac = cloud.size() == 1
                            ? 0.0
                            : static_cast<double>(i) / static_cast<double>(cloud.size() - 1);
    ts1.emplace_back(Secondsd{FIRST_SCAN_END * frac});
  }
  lio.register_scan(cloud, ts1);

  // Static IMU samples between the two scans. Use enough samples that interval_stats
  // is well-formed for the inter-scan filter.
  constexpr int N = 10;
  for (int i = 0; i < N; ++i) {
    ImuControl m;
    const double frac = static_cast<double>(i) / static_cast<double>(N - 1);
    m.time = Secondsd{FIRST_SCAN_END + 0.05 + (DT - 0.1) * frac};
    m.acceleration = {0.0, 0.0, GRAVITY_MAG};
    m.angular_velocity = Eigen::Vector3d::Zero();
    lio.add_imu_measurement(m);
  }

  // Single-instant timestamps for the second scan -> deskewing is identity.
  const TimestampVector ts2(cloud.size(), Secondsd{SECOND_SCAN_END});
  lio.register_scan(cloud, ts2);

  REQUIRE(lio.poses_with_timestamps.size() == 2);
  REQUIRE(approx_equal(lio.imu_state.pose, lio.lidar_state.pose, 1e-12));
  REQUIRE_THAT(lio.imu_state.time.count(), WithinAbs(lio.lidar_state.time.count(), 1e-12));
  REQUIRE(approx_equal(lio.imu_state.velocity, lio.lidar_state.velocity, 1e-12));
  REQUIRE(approx_equal(lio.imu_state.angular_velocity, lio.lidar_state.angular_velocity, 1e-12));
  REQUIRE(approx_equal(lio.imu_state.linear_acceleration, lio.lidar_state.linear_acceleration, 1e-12));
}
