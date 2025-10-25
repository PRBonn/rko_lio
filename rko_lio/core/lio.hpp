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
 * Core LIO class and utilities for RKO-LIO.
 */

#pragma once
#include "sparse_voxel_grid.hpp"
#include "util.hpp"
#include <optional>

/** Core namespace containing LIO data structures and state definitions. */
namespace rko_lio::core {
/** Return type for the acceleration Kalman filter containing gravity and variance estimates. */
struct AccelInfo {
  /** Variance of the raw imu acceleration magnitude. */
  double accel_mag_variance;
  Eigen::Vector3d local_gravity_estimate;
};

/** Accumulated IMU statistics over the interval between consecutive LiDAR scans. */
struct IntervalStats {
  /** Number of IMU samples accumulated during this interval. */
  int imu_count = 0;

  /** Sum of unbiased angular velocity samples. */
  Eigen::Vector3d angular_velocity_sum = Eigen::Vector3d::Zero();

  /** Sum of gravity-compensated body-frame accelerations. */
  Eigen::Vector3d body_acceleration_sum = Eigen::Vector3d::Zero();

  /** Sum of unbiased raw IMU accelerations. */
  Eigen::Vector3d imu_acceleration_sum = Eigen::Vector3d::Zero();

  /** Mean magnitude of raw IMU acceleration over the interval. */
  double imu_accel_mag_mean = 0;

  /** Variance accumulator for acceleration magnitude using Welford’s method. */
  double welford_sum_of_squares = 0;

  /**
   * Update accumulated statistics with a new IMU measurement.
   * @param unbiased_ang_vel Unbiased angular velocity.
   * @param uncompensated_unbiased_accel Uncompensated, unbiased acceleration.
   * @param compensated_accel Gravity-compensated acceleration.
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

  /** Reset all accumulated statistics to zero. */
  void reset() {
    imu_count = 0;
    angular_velocity_sum.setZero();
    body_acceleration_sum.setZero();
    imu_acceleration_sum.setZero();
    imu_accel_mag_mean = 0;
    welford_sum_of_squares = 0;
  }
};

/** RKO-LIO's core LiDAR-inertial odometry algorithm class. */
class LIO {
public:
  /** Configuration parameters for odometry. */
  struct Config {
    /** Enable scan deskewing. */
    bool deskew = true;

    /** Maximum number of ICP iterations. */
    size_t max_iterations = 100;

    /** Size of voxel grid (m). */
    double voxel_size = 1.0;

    /** Max points per voxel. */
    int max_points_per_voxel = 20;

    /** Maximum lidar range (m). */
    double max_range = 100.0;

    /** Minimum lidar range (m). */
    double min_range = 1.0;

    /** ICP convergence threshold. */
    double convergence_criterion = 1e-5;

    /** Max distance for correspondences (m). */
    double max_correspondance_distance = 0.5;

    /** Thread count for data association (0 = automatic). */
    int max_num_threads = 0;

    /** Enable initialization phase. */
    bool initialization_phase = false;

    /** Maximum expected jerk (m/s³). */
    double max_expected_jerk = 3;

    /** Enable double downsampling. */
    bool double_downsample = true;

    /** Minimum weight for orientation regularization. */
    double min_beta = 200;
  };

  /** Configuration parameters. */
  Config config;

  /** Local map as sparse voxel grid (Bonxai). */
  SparseVoxelGrid map;

  /** Current LiDAR state estimate. */
  State lidar_state;

  /** IMU bias estimates when initialization is enabled. */
  ImuBias imu_bias;

  /** Mean body acceleration estimate. */
  Eigen::Vector3d mean_body_acceleration = Eigen::Vector3d::Zero();

  /** Covariance of body acceleration estimate. */
  Eigen::Matrix3d body_acceleration_covariance = Eigen::Matrix3d::Identity();

  /** IMU measurement statistics since last LiDAR frame. */
  IntervalStats interval_stats;

  explicit LIO(const Config& config_)
      : config(config_), map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel) {}

  /** Add an IMU measurement expressed in the base frame. */
  void add_imu_measurement(const ImuControl& base_imu);

  /**
   * Add an IMU measurement expressed in the IMU frame and transform it
   * to the base frame using the given extrinsic calibration.
   * @param extrinsic_imu2base Extrinsic transform from IMU to base frame.
   * @param raw_imu Raw IMU measurement.
   */
  void add_imu_measurement(const Sophus::SE3d& extrinsic_imu2base, const ImuControl& raw_imu);

  /**
   * Register a LiDAR scan, applying deskewing based on the initial motion guess
   * and clipping points beyond valid range.
   * @param scan Input raw point cloud.
   * @param timestamps Absolute timestamps corresponding to each scan point.
   * @return Deskewed and clipped point cloud.
   */
  Vector3dVector register_scan(const Vector3dVector& scan, const TimestampVector& timestamps);

  /**
   * Register a LiDAR scan for which the extrinsic calibration from lidar to base
   * has already been applied.
   * @param extrinsic_lidar2base Extrinsic from lidar to base frame.
   * @param scan Input raw point cloud.
   * @param timestamps Absolute timestamps corresponding to each scan point.
   * @return Deskewed and clipped scan in the original lidar frame.
   */
  Vector3dVector register_scan(const Sophus::SE3d& extrinsic_lidar2base,
                               const Vector3dVector& scan,
                               const TimestampVector& timestamps);

  /** Sequence of registered scan poses with corresponding timestamps. */
  std::vector<std::pair<Secondsd, Sophus::SE3d>> poses_with_timestamps;

private:
  /**
   * Initialize internal odometry state using the given lidar timestamp.
   * @param lidar_time Current lidar timestamp.
   */
  void initialize(const Secondsd lidar_time);

  /** get the convenience struct with accel mag variance and local gravity estimate. */
  std::optional<AccelInfo> get_accel_info(const Sophus::SO3d& rotation_estimate, const Secondsd& time);

  /** True if odometry initialization has been completed. */
  bool _initialized = false;

  /** Latest IMU orientation used for gravity compensation. This is ahead of the rotation in the state. */
  Sophus::SO3d _imu_local_rotation;

  /** Timestamp of the latest IMU orientation. Once a scan is registered, this is reset to the lidar state orientation.
   */
  Secondsd _imu_local_rotation_time = Secondsd{0.0};

  /** Timestamp of the most recent real IMU measurement. */
  Secondsd _last_real_imu_time = Secondsd{0.0};

  /** Angular velocity of last true IMU measurement expressed in base frame. */
  Eigen::Vector3d _last_real_base_imu_ang_vel = Eigen::Vector3d::Zero();
};
} // namespace rko_lio::core
