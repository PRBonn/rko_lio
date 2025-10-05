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
/**
 * @file lio.hpp
 * @brief Core LIO class and utilities for RKO-LIO
 */
#pragma once
#include "sparse_voxel_grid.hpp"
#include "util.hpp"
#include <filesystem>
#include <optional>

/**
 * @namespace rko_lio::core
 * @brief Core namespace containing LIO data structures
 */
namespace rko_lio::core {
/**
 * @brief Return type struct for use in the acceleration Kalman Filter
 */
struct AccelInfo {
  double accel_mag_variance;
  Eigen::Vector3d local_gravity_estimate;
};

/**
 * @brief Accumulated IMU data stats over the time interval from previous lidar scan to current
 */
struct IntervalStats {
  int imu_count = 0;                                               ///< Number of IMU samples accumulated
  Eigen::Vector3d angular_velocity_sum = Eigen::Vector3d::Zero();  ///< Sum of angular velocities
  Eigen::Vector3d body_acceleration_sum = Eigen::Vector3d::Zero(); ///< Sum of gravity compensated accelerations
  Eigen::Vector3d imu_acceleration_sum = Eigen::Vector3d::Zero();  ///< Sum of raw IMU accelerations
  double imu_accel_mag_mean = 0;                                   ///< Mean acceleration magnitude over interval
  double welford_sum_of_squares = 0;                               ///< Variance accumulator for acceleration magnitude

  /**
   * @brief Update stats with a new IMU measurement
   */
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

  /**
   * @brief Reset all stats to zero
   */
  void reset() {
    imu_count = 0;
    angular_velocity_sum.setZero();
    body_acceleration_sum.setZero();
    imu_acceleration_sum.setZero();
    imu_accel_mag_mean = 0;
    welford_sum_of_squares = 0;
  }
};

/**
 * @brief RKO-LIO's core LiDAR-inertial odometry algorithm class
 */
class LIO {
public:
  /**
   * @brief Configuration parameters for odometry
   */
  struct Config {
    bool deskew = true;                       ///< Enable scan deskewing
    size_t max_iterations = 100;              ///< Maximum number of ICP iterations
    double voxel_size = 1.0;                  ///< Size of voxel grid (meters)
    int max_points_per_voxel = 20;            ///< Max points per voxel
    double max_range = 100.0;                 ///< Max lidar range (meters)
    double min_range = 1.0;                   ///< Min lidar range (meters)
    double convergence_criterion = 1e-5;      ///< ICP convergence threshold
    double max_correspondance_distance = 0.5; ///< Max distance for point correspondences (meters)
    int max_num_threads = 0;                  ///< Thread count used for data associations, zero for automatic
    bool initialization_phase = false;        ///< True to enable initialization phase
    double max_expected_jerk = 3;             ///< Maximum expected jerk for platform (m/s³)
    bool double_downsample = true;            ///< Use double downsampling
    double min_beta = 200;                    ///< Minimum weight for orientation regularisation
  };

  Config config;
  SparseVoxelGrid map;
  State lidar_state; ///< Current lidar pose/state estimate
  ImuBias imu_bias;  ///< IMU bias estimates if initialization was enabled

  Eigen::Vector3d mean_body_acceleration = Eigen::Vector3d::Zero(); ///< Estimated body acceleration
  Eigen::Matrix3d body_acceleration_covariance =
      Eigen::Matrix3d::Identity(); ///< Covariance of body acceleration estimate

  IntervalStats interval_stats;

  explicit LIO(const Config& config_)
      : config(config_), map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel) {}

  /**
   * @brief Add an IMU measurement in base frame
   * @param base_imu IMU measurement in base frame
   */
  void add_imu_measurement(const ImuControl& base_imu);
  /**
   * @brief Add an IMU measurement in IMU frame and transform it by the extrinsic calibration
   * @param extrinsic_imu2base Extrinsic transform from IMU to base frame
   * @param raw_imu Raw IMU measurement
   */
  void add_imu_measurement(const Sophus::SE3d& extrinsic_imu2base, const ImuControl& raw_imu);

  // returns deskewed (only using initial motion guess) and clipped scan
  /**
   * @brief Register a scan
   * @param scan raw point cloud
   * @param timestamps Corresponding absolute timestamps for each scan point
   * @return Deskewed and clipped scan
   */
  Vector3dVector register_scan(const Vector3dVector& scan, const TimestampVector& timestamps);
  /**
   * @brief Register a scan with extrinsic pre-applied
   * @param extrinsic_lidar2base Extrinsic from lidar to base frame
   * @param scan raw point cloud
   * @param timestamps Corresponding absolute timestamps for each scan point
   * @return Deskewed and clipped scan in the original lidar frame
   */
  Vector3dVector register_scan(const Sophus::SE3d& extrinsic_lidar2base,
                               const Vector3dVector& scan,
                               const TimestampVector& timestamps);

  /**
   * @brief Dump trajectory results to disk
   * @param results_dir Directory where results are saved
   * @param run_name Identifier for run/folder name
   */
  void dump_results_to_disk(const std::filesystem::path& results_dir, const std::string& run_name) const;

private:
  /**
   * @brief Initialize internal state with lidar time
   * @param lidar_time Current lidar timestamp
   */
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
