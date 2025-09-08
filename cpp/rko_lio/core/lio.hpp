/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once
#include "sparse_voxel_grid.hpp"
#include "util.hpp"
#include <filesystem>

namespace rko_lio::core {
// return type struct in place of a tuple
struct AccelInfo {
  double accel_mag_variance;
  Eigen::Vector3d local_gravity_estimate;
};

struct IntervalStats {
  int imu_count = 0;
  Eigen::Vector3d angular_velocity_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_acceleration_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_acceleration_sum = Eigen::Vector3d::Zero();
  double imu_accel_mag_mean = 0;
  double welford_sum_of_squares = 0;

  void update(const Eigen::Vector3d& unbiased_ang_vel,
              const Eigen::Vector3d& uncompensated_unbiased_accel,
              const Eigen::Vector3d& compensated_accel) {
    ++imu_count;
    angular_velocity_sum += unbiased_ang_vel;
    imu_acceleration_sum += uncompensated_unbiased_accel;

    const double previous_mean = imu_accel_mag_mean;
    const double accel_norm = uncompensated_unbiased_accel.norm();

    imu_accel_mag_mean += (accel_norm - previous_mean) / imu_count;
    welford_sum_of_squares += (accel_norm - previous_mean) * (accel_norm - imu_accel_mag_mean);

    body_acceleration_sum += compensated_accel;
  }
  void reset() {
    imu_count = 0;
    angular_velocity_sum.setZero();
    body_acceleration_sum.setZero();
    imu_acceleration_sum.setZero();
    imu_accel_mag_mean = 0;
    welford_sum_of_squares = 0;
  }
};

class LIO {
public:
  struct Config {
    bool deskew = true;
    size_t max_iterations = 100;
    double voxel_size = 1.0; // m
    int max_points_per_voxel = 20;
    double max_range = 100.0; // m
    double min_range = 1.0;   // m
    double convergence_criterion = 1e-5;
    double max_correspondance_distance = 0.5; // m
    int max_num_threads = 0;
    bool initialization_phase = true;
    double max_expected_jerk = 3; // m/s3
    bool double_downsample = true;
    double min_beta = 200;
  };

  Config config;
  SparseVoxelGrid map;
  State lidar_state;
  ImuBias imu_bias;
  Eigen::Vector3d mean_body_acceleration = Eigen::Vector3d::Zero();
  Eigen::Matrix3d body_acceleration_covariance = Eigen::Matrix3d::Identity();

  IntervalStats interval_stats;

  explicit LIO(const Config& config_)
      : config(config_), map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel) {}

  void add_imu_measurement(const ImuControl& base_imu);
  // Pre-transform the data by the extrinsic
  void add_imu_measurement(const Sophus::SE3d& extrinsic_imu2base, const ImuControl& raw_imu);

  // returns deskewed (only using initial motion guess) and clipped scan
  Vector3dVector register_scan(const Vector3dVector& scan, const TimestampVector& timestamps);
  // Pre-transform cloud by extrinsic - lidar to base. The return is still in the original frame
  Vector3dVector register_scan(const Sophus::SE3d& extrinsic_lidar2base,
                               const Vector3dVector& scan,
                               const TimestampVector& timestamps);

  void dump_results_to_disk(const std::filesystem::path& results_dir, const std::string& run_name) const;

private:
  void initialize(const Secondsd lidar_time);

  std::optional<AccelInfo> get_accel_info(const Sophus::SO3d& rotation_estimate, const Secondsd& time);

  bool _initialized = false;
  // rotation used for gravity compensating incoming accel
  Sophus::SO3d _imu_local_rotation;
  Secondsd _imu_local_rotation_time = Secondsd{0.0}; // updated with lidar time on correction
  Secondsd _last_real_imu_time = Secondsd{0.0};
  Eigen::Vector3d _last_real_base_imu_ang_vel = Eigen::Vector3d::Zero();

  std::vector<std::pair<Secondsd, Sophus::SE3d>> _poses_with_timestamps;
};
} // namespace rko_lio::core
