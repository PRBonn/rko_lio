#include "eigen_approx.hpp"
#include "rko_lio/core/lio.hpp"
#include "synthetic_clouds.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>

using Catch::Matchers::WithinAbs;
using rko_lio::core::GRAVITY_MAG;
using rko_lio::core::ImuControl;
using rko_lio::core::LIO;
using rko_lio::core::Nsec;
using rko_lio::core::Timestamps;
using rko_lio::core::TimestampVector;
using rko_lio::core::to_seconds;
using rko_lio::core::Vector3sVector;
using rko_lio::tests::approx_equal;
using rko_lio::tests::EXACT_TOL;
using rko_lio::tests::make_hollow_cube;

namespace {
inline Nsec ns_from_seconds(double s) { return Nsec(static_cast<int64_t>(std::llround(s * 1e9))); }
} // namespace

namespace {
// dt between consecutive scans, used to derive the IMU values that produce a
// given initial-guess pose. With v0 = 0 the kinematic model collapses to
//   tau.head = a * DT^2 / 2,  tau.tail = omega * DT,  initial_guess = SE3::exp(tau).
constexpr double DT = 1.0;
constexpr double FIRST_SCAN_END = 0.1;
constexpr double SECOND_SCAN_END = FIRST_SCAN_END + DT;

Timestamps linspace_timestamps(size_t n, double t_start, double t_end) {
  TimestampVector ts;
  ts.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const double frac = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    ts.emplace_back(ns_from_seconds(t_start + (t_end - t_start) * frac));
  }
  const Nsec t_start_ns = ns_from_seconds(t_start);
  const Nsec t_end_ns = ns_from_seconds(t_end);
  return Timestamps{
      .min = std::min(t_start_ns, t_end_ns), .max = std::max(t_start_ns, t_end_ns), .per_point = std::move(ts)};
}

// Single-instant timestamps for the second scan: deskewing reduces to identity
// when every point shares end_time.
Timestamps instant_timestamps(size_t n, double t) {
  return Timestamps{
      .min = ns_from_seconds(t), .max = ns_from_seconds(t), .per_point = TimestampVector(n, ns_from_seconds(t))};
}

void feed_imu(LIO& lio,
              double t_start,
              double t_end,
              int n,
              const Eigen::Vector3s& acceleration,
              const Eigen::Vector3s& angular_velocity) {
  for (int i = 0; i < n; ++i) {
    ImuControl m;
    const double frac = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    m.time = ns_from_seconds(t_start + (t_end - t_start) * frac);
    m.acceleration = acceleration;
    m.angular_velocity = angular_velocity;
    lio.add_imu_measurement(m);
  }
}

void feed_static_imu(LIO& lio, double t_start, double t_end, int n) {
  feed_imu(lio, t_start, t_end, n, {0.0, 0.0, GRAVITY_MAG}, Eigen::Vector3s::Zero());
}

// Same as feed_imu but with per-sample Gaussian noise added to accel and gyro.
// Seed is fixed at the call site so the test is deterministic.
void feed_imu_noisy(LIO& lio,
                    double t_start,
                    double t_end,
                    int n,
                    const Eigen::Vector3s& acceleration,
                    const Eigen::Vector3s& angular_velocity,
                    double accel_stddev,
                    double gyro_stddev,
                    uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> a_noise(0.0, accel_stddev);
  std::normal_distribution<double> g_noise(0.0, gyro_stddev);
  for (int i = 0; i < n; ++i) {
    ImuControl m;
    const double frac = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    m.time = ns_from_seconds(t_start + (t_end - t_start) * frac);
    m.acceleration = acceleration + Eigen::Vector3s(a_noise(rng), a_noise(rng), a_noise(rng));
    m.angular_velocity = angular_velocity + Eigen::Vector3s(g_noise(rng), g_noise(rng), g_noise(rng));
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

  REQUIRE(lio.map.empty());
  REQUIRE(lio.poses_with_timestamps.empty());

  lio.register_scan(cloud, ts);

  REQUIRE_FALSE(lio.map.empty());
  REQUIRE(lio.poses_with_timestamps.size() == 1);
  REQUIRE_THAT(to_seconds(lio.lidar_state.time), WithinAbs(FIRST_SCAN_END, 1e-9));
  REQUIRE(approx_equal(lio.lidar_state.pose, Sophus::SE3s{}, EXACT_TOL));
  REQUIRE_THAT(to_seconds(lio.poses_with_timestamps[0].first), WithinAbs(FIRST_SCAN_END, 1e-9));
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
  const Eigen::Vector3s t(1.0, 0.0, 0.0);
  const Eigen::Vector3s a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3s imu_accel = a_body + Eigen::Vector3s{0.0, 0.0, GRAVITY_MAG};
  feed_imu(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel, Eigen::Vector3s::Zero());

  Vector3sVector cloud2;
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
  const Eigen::Vector3s omega(0.0, 0.0, yaw / DT);
  feed_imu(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, {0.0, 0.0, GRAVITY_MAG}, omega);

  const Sophus::SO3s R = Sophus::SO3s::rotZ(yaw);
  const Sophus::SO3s R_inv = R.inverse();
  Vector3sVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(R_inv * p);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  const auto& pose = lio.lidar_state.pose;
  REQUIRE(approx_equal(pose.so3(), R, 1e-3));
  REQUIRE(pose.translation().norm() < 1e-3);
}

TEST_CASE("Gap in the lidar stream: prior extrapolates instead of throwing", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  // a gap longer than the one second tolerance, with the imu still streaming. yawing about the gravity axis keeps the
  // gravity compensation exact, so the body acceleration in x turns into a real translation as well as a rotation
  constexpr double SECOND_SCAN_START = FIRST_SCAN_END + 1.5;
  feed_imu(lio, FIRST_SCAN_END, SECOND_SCAN_START, 150, {2.0, 0.0, GRAVITY_MAG}, {0.0, 0.0, 0.4});

  // the imu integration is where that motion actually ended up, so put the cloud there and let the prior find it
  const Sophus::SE3s ground_truth = lio.imu_state.pose;
  REQUIRE(ground_truth.translation().norm() > 1.0);
  REQUIRE(ground_truth.so3().log().norm() > 0.1);

  const Sophus::SE3s into_ground_truth_frame = ground_truth.inverse();
  Vector3sVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(into_ground_truth_frame * p);
  }
  REQUIRE_NOTHROW(lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_START)));

  REQUIRE(lio.poses_with_timestamps.size() == 2);
  REQUIRE(approx_equal(lio.lidar_state.pose, ground_truth, 1e-2));
}

TEST_CASE("Gap in the lidar stream: second scan has real per-point spread, not instant", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube(2.0, 5.0);

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  constexpr double SECOND_SCAN_START = FIRST_SCAN_END + 1.5;
  constexpr double SECOND_SCAN_SPAN_END = SECOND_SCAN_START + 0.1;
  const Eigen::Vector3s a_body(2.0, 0.0, 0.0);
  const Eigen::Vector3s omega(0.0, 0.0, 0.4);
  const Eigen::Vector3s imu_accel = a_body + Eigen::Vector3s{0.0, 0.0, GRAVITY_MAG};
  feed_imu(lio, FIRST_SCAN_END, SECOND_SCAN_START, 150, imu_accel, omega);

  // one imu sample per point, at that point's own timestamp; imu_state right after is a true
  // per-sample-integrated pose for that point, independent of deskew's own (coarser) model
  const Timestamps ts2 = linspace_timestamps(cloud.size(), SECOND_SCAN_START, SECOND_SCAN_SPAN_END);
  Vector3sVector cloud2(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    ImuControl m;
    m.time = ts2.per_point[i];
    m.acceleration = imu_accel;
    m.angular_velocity = omega;
    lio.add_imu_measurement(m);
    cloud2[i] = lio.imu_state.pose.inverse() * cloud[i];
  }
  const Sophus::SE3s ground_truth = lio.imu_state.pose;
  REQUIRE(ground_truth.translation().norm() > 1.0);
  REQUIRE(ground_truth.so3().log().norm() > 0.1);

  REQUIRE_NOTHROW(lio.register_scan(cloud2, ts2));

  REQUIRE(lio.poses_with_timestamps.size() == 2);
  REQUIRE(approx_equal(lio.lidar_state.pose, ground_truth, 1e-2));
}

TEST_CASE("Full SE(3): translation + rotation combined", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();

  lio.register_scan(cloud, linspace_timestamps(cloud.size(), 0.0, FIRST_SCAN_END));

  // Combined yaw + acceleration
  const double yaw = 3.0 * M_PI / 180.0;
  const Eigen::Vector3s t(0.5, 0.3, 0.0);
  const Eigen::Vector3s a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3s imu_accel = a_body + Eigen::Vector3s{0.0, 0.0, GRAVITY_MAG};
  const Eigen::Vector3s omega(0.0, 0.0, yaw / DT);
  feed_imu(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel, omega);

  const Sophus::SE3s T_expected(Sophus::SO3s::rotZ(yaw), t);
  const Sophus::SE3s T_inv = T_expected.inverse();
  Vector3sVector cloud2;
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

  const Eigen::Vector3s t(1.0, 0.0, 0.0);
  const Eigen::Vector3s a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3s imu_accel = a_body + Eigen::Vector3s{0.0, 0.0, GRAVITY_MAG};
  feed_imu_noisy(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel, Eigen::Vector3s::Zero(),
                 NOISY_ACCEL_STDDEV, NOISY_GYRO_STDDEV, /*seed=*/1u);

  Vector3sVector cloud2;
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
  const Eigen::Vector3s omega(0.0, 0.0, yaw / DT);
  feed_imu_noisy(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, {0.0, 0.0, GRAVITY_MAG}, omega,
                 NOISY_ACCEL_STDDEV, NOISY_GYRO_STDDEV,
                 /*seed=*/2u);

  const Sophus::SO3s R = Sophus::SO3s::rotZ(yaw);
  const Sophus::SO3s R_inv = R.inverse();
  Vector3sVector cloud2;
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
  const Eigen::Vector3s t(0.5, 0.3, 0.0);
  const Eigen::Vector3s a_body = 2.0 * t / (DT * DT);
  const Eigen::Vector3s imu_accel = a_body + Eigen::Vector3s{0.0, 0.0, GRAVITY_MAG};
  const Eigen::Vector3s omega(0.0, 0.0, yaw / DT);
  feed_imu_noisy(lio, FIRST_SCAN_END + 0.05, SECOND_SCAN_END - 0.05, 10, imu_accel, omega, NOISY_ACCEL_STDDEV,
                 NOISY_GYRO_STDDEV, /*seed=*/3u);

  const Sophus::SE3s T_expected(Sophus::SO3s::rotZ(yaw), t);
  const Sophus::SE3s T_inv = T_expected.inverse();
  Vector3sVector cloud2;
  cloud2.reserve(cloud.size());
  for (const auto& p : cloud) {
    cloud2.emplace_back(T_inv * p);
  }
  lio.register_scan(cloud2, instant_timestamps(cloud2.size(), SECOND_SCAN_END));

  CAPTURE(lio.lidar_state.pose.translation().transpose(), lio.lidar_state.pose.so3().log().transpose());
  CHECK(approx_equal(lio.lidar_state.pose, T_expected, 1e-2));
}

TEST_CASE("register_scan: empty timestamps throws instead of UB", "[register_scan]") {
  LIO lio((LIO::Config{}));
  const auto cloud = make_hollow_cube();
  const Timestamps empty_timestamps{};
  REQUIRE_THROWS_AS(lio.register_scan(cloud, empty_timestamps), std::invalid_argument);
}
