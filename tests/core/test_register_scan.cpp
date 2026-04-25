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
#include "synthetic_clouds.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <random>

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
// dt between consecutive scans, used to derive the IMU values that produce a
// given initial-guess pose. With v0 = 0 the kinematic model collapses to
//   tau.head = a * DT^2 / 2,  tau.tail = omega * DT,  initial_guess = SE3::exp(tau).
constexpr double DT = 1.0;
constexpr double FIRST_SCAN_END = 0.1;
constexpr double SECOND_SCAN_END = FIRST_SCAN_END + DT;

TimestampVector linspace_timestamps(size_t n, double t_start, double t_end) {
  TimestampVector ts;
  ts.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const double frac = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    ts.emplace_back(Secondsd{t_start + (t_end - t_start) * frac});
  }
  return ts;
}

// Single-instant timestamps for the second scan: deskewing reduces to identity
// when every point shares end_time.
TimestampVector instant_timestamps(size_t n, double t) {
  return TimestampVector(n, Secondsd{t});
}

void feed_imu(LIO& lio, double t_start, double t_end, int n,
              const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity) {
  for (int i = 0; i < n; ++i) {
    ImuControl m;
    const double frac = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    m.time = Secondsd{t_start + (t_end - t_start) * frac};
    m.acceleration = acceleration;
    m.angular_velocity = angular_velocity;
    lio.add_imu_measurement(m);
  }
}

void feed_static_imu(LIO& lio, double t_start, double t_end, int n) {
  feed_imu(lio, t_start, t_end, n, {0.0, 0.0, GRAVITY_MAG}, Eigen::Vector3d::Zero());
}

// Same as feed_imu but with per-sample Gaussian noise added to accel and gyro.
// Seed is fixed at the call site so the test is deterministic.
void feed_imu_noisy(LIO& lio, double t_start, double t_end, int n,
                    const Eigen::Vector3d& acceleration, const Eigen::Vector3d& angular_velocity,
                    double accel_stddev, double gyro_stddev, uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> a_noise(0.0, accel_stddev);
  std::normal_distribution<double> g_noise(0.0, gyro_stddev);
  for (int i = 0; i < n; ++i) {
    ImuControl m;
    const double frac = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    m.time = Secondsd{t_start + (t_end - t_start) * frac};
    m.acceleration = acceleration + Eigen::Vector3d{a_noise(rng), a_noise(rng), a_noise(rng)};
    m.angular_velocity = angular_velocity + Eigen::Vector3d{g_noise(rng), g_noise(rng), g_noise(rng)};
    lio.add_imu_measurement(m);
  }
}

// Noise levels for the [!mayfail] robustness variants. Small enough that ICP
// should still close the gap to ground truth; large enough to perturb the
// IMU-derived initial guess noticeably.
constexpr double NOISY_ACCEL_STDDEV = 0.05; // m/s^2
constexpr double NOISY_GYRO_STDDEV = 5e-3;  // rad/s
} // namespace

TEST_CASE("First scan: empty map -> populated, single pose at lidar_state.time", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();
  const auto ts = linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END);

  REQUIRE(lio.map.Empty());
  REQUIRE(lio.poses_with_timestamps.empty());

  lio.register_scan(cloud, ts);

  REQUIRE_FALSE(lio.map.Empty());
  REQUIRE(lio.poses_with_timestamps.size() == 1);
  REQUIRE_THAT(lio.lidar_state.time.count(), WithinAbs(FIRST_SCAN_END, 1e-9));
  REQUIRE(approx_equal(lio.lidar_state.pose, Sophus::SE3d{}, 1e-12));
  REQUIRE_THAT(lio.poses_with_timestamps[0].first.count(), WithinAbs(FIRST_SCAN_END, 1e-9));
}

TEST_CASE("Identity registration: same cloud twice -> pose ~= identity", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));
  feed_static_imu(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10);
  lio.register_scan(cloud, instant_timestamps(cloud.size(), SECOND_SCAN_END));

  REQUIRE(lio.poses_with_timestamps.size() == 2);
  REQUIRE(lio.lidar_state.pose.translation().norm() < 1e-3);
  REQUIRE(lio.lidar_state.pose.so3().log().norm() < 1e-3);
}

TEST_CASE("deskew = false: register_scan still produces a valid pose", "[register_scan]") {
  LIO::Config cfg{};
  cfg.deskew = false;
  LIO lio(cfg);
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));
  feed_static_imu(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10);
  lio.register_scan(cloud, instant_timestamps(cloud.size(), SECOND_SCAN_END));

  REQUIRE(lio.poses_with_timestamps.size() == 2);
  REQUIRE(lio.lidar_state.pose.translation().norm() < 1e-3);
  REQUIRE(lio.lidar_state.pose.so3().log().norm() < 1e-3);
}

TEST_CASE("Pure translation: recover +1m in x", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  // Body accel a such that a * DT^2 / 2 = 1m in x  =>  a = 2 m/s^2. Add gravity for
  // the raw IMU value so compensated body accel comes out to (a, 0, 0).
  const Eigen::Vector3d t(1.0, 0.0, 0.0);
  const Eigen::Vector3d a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3d imu_accel = a_body + Eigen::Vector3d{0.0, 0.0, GRAVITY_MAG};
  feed_imu(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel, Eigen::Vector3d::Zero());

  Vector3dVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(p - t);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  const auto& pose = lio.lidar_state.pose;
  REQUIRE(approx_equal(pose.translation(), t, 1e-3));
  REQUIRE(pose.so3().log().norm() < 1e-3);
}

TEST_CASE("Pure rotation: recover 5 deg yaw", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  // omega * DT = yaw (5 deg) about z  =>  omega = yaw/DT rad/s.
  const double yaw = 5.0 * M_PI / 180.0;
  const Eigen::Vector3d omega(0.0, 0.0, yaw / DT);
  feed_imu(lio,
           FIRST_SCAN_END + 0.05,
           SECOND_SCAN_END - 0.05,
           10,
           {0.0, 0.0, GRAVITY_MAG},
           omega);

  const Sophus::SO3d R = Sophus::SO3d::rotZ(yaw);
  const Sophus::SO3d R_inv = R.inverse();
  Vector3dVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(R_inv * p);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  const auto& pose = lio.lidar_state.pose;
  REQUIRE(approx_equal(pose.so3(), R, 1e-3));
  REQUIRE(pose.translation().norm() < 1e-3);
}

TEST_CASE("Full SE(3): translation + rotation combined", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  // Combined yaw + acceleration
  const double yaw = 3.0 * M_PI / 180.0;
  const Eigen::Vector3d t(0.5, 0.3, 0.0);
  const Eigen::Vector3d a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3d imu_accel = a_body + Eigen::Vector3d{0.0, 0.0, GRAVITY_MAG};
  const Eigen::Vector3d omega(0.0, 0.0, yaw / DT);
  feed_imu(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel, omega);

  const Sophus::SE3d T_expected(Sophus::SO3d::rotZ(yaw), t);
  const Sophus::SE3d T_inv = T_expected.inverse();
  Vector3dVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(T_inv * p);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  REQUIRE(approx_equal(lio.lidar_state.pose, T_expected, 1e-2));
}

// ===========================================================================
// Noisy variants. The IMU-derived initial guess is perturbed by Gaussian noise;
// ICP has to refine it. Tagged [!mayfail] so a borderline failure surfaces in
// the report without breaking the suite -- drop the tag to make these strict.
// ===========================================================================

TEST_CASE("Noisy: pure translation +1m in x", "[register_scan][!mayfail]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  const Eigen::Vector3d t(1.0, 0.0, 0.0);
  const Eigen::Vector3d a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3d imu_accel = a_body + Eigen::Vector3d{0.0, 0.0, GRAVITY_MAG};
  feed_imu_noisy(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel,
                 Eigen::Vector3d::Zero(), NOISY_ACCEL_STDDEV, NOISY_GYRO_STDDEV, /*seed=*/1u);

  Vector3dVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(p - t);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  const auto& pose = lio.lidar_state.pose;
  CAPTURE(pose.translation().transpose(), pose.so3().log().transpose());
  CHECK(approx_equal(pose.translation(), t, 1e-3));
  CHECK(pose.so3().log().norm() < 1e-3);
}

TEST_CASE("Noisy: pure rotation 5 deg yaw", "[register_scan][!mayfail]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  const double yaw = 5.0 * M_PI / 180.0;
  const Eigen::Vector3d omega(0.0, 0.0, yaw / DT);
  feed_imu_noisy(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10,
                 {0.0, 0.0, GRAVITY_MAG}, omega, NOISY_ACCEL_STDDEV, NOISY_GYRO_STDDEV,
                 /*seed=*/2u);

  const Sophus::SO3d R = Sophus::SO3d::rotZ(yaw);
  const Sophus::SO3d R_inv = R.inverse();
  Vector3dVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(R_inv * p);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  const auto& pose = lio.lidar_state.pose;
  CAPTURE(pose.translation().transpose(), pose.so3().log().transpose());
  CHECK(approx_equal(pose.so3(), R, 1e-3));
  CHECK(pose.translation().norm() < 1e-3);
}

TEST_CASE("Noisy: full SE(3) translation + rotation", "[register_scan][!mayfail]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  const double yaw = 3.0 * M_PI / 180.0;
  const Eigen::Vector3d t(0.5, 0.3, 0.0);
  const Eigen::Vector3d a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3d imu_accel = a_body + Eigen::Vector3d{0.0, 0.0, GRAVITY_MAG};
  const Eigen::Vector3d omega(0.0, 0.0, yaw / DT);
  feed_imu_noisy(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel, omega,
                 NOISY_ACCEL_STDDEV, NOISY_GYRO_STDDEV, /*seed=*/3u);

  const Sophus::SE3d T_expected(Sophus::SO3d::rotZ(yaw), t);
  const Sophus::SE3d T_inv = T_expected.inverse();
  Vector3dVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(T_inv * p);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  CAPTURE(lio.lidar_state.pose.translation().transpose(),
          lio.lidar_state.pose.so3().log().transpose());
  CHECK(approx_equal(lio.lidar_state.pose, T_expected, 1e-2));
}
