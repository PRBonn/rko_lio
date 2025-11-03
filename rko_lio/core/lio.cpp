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

#include "lio.hpp"
#include "preprocess_scan.hpp"
#include "profiler.hpp"
#include "util.hpp"
// other
#include <sophus/se3.hpp>
// tbb
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
// stl
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace {
// The only parallel part. taken from kiss-icp
// correspondence - original point and corresponding map point, in that order
using OneCorrespondence = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using Correspondences = tbb::concurrent_vector<OneCorrespondence>;
Correspondences data_association(const Sophus::SE3d& pose,
                                 const rko_lio::core::Vector3dVector& points,
                                 const rko_lio::core::SparseVoxelGrid& voxel_map,
                                 const rko_lio::core::LIO::Config& config) {
  const int max_threads = config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  static const auto tbb_control_settings =
      tbb::global_control(tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_threads));
  using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
  Correspondences correspondences;
  correspondences.reserve(points.size());
  tbb::parallel_for(tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
                    [&](const tbb::blocked_range<points_iterator>& r) {
                      std::for_each(r.begin(), r.end(), [&](const auto& point) {
                        // TODO: left jacobian means we can reduce some compute here
                        // transform the source point here and get the corresponding map point
                        const auto& [closest_neighbor, distance] = voxel_map.GetClosestNeighbor(pose * point);
                        if (distance < config.max_correspondance_distance) {
                          correspondences.emplace_back(point, closest_neighbor);
                        }
                      });
                    });
  return correspondences;
}
} // namespace

namespace {
constexpr double EPSILON = 1e-8;
constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);
using namespace rko_lio::core;

inline void transform_points(const Sophus::SE3d& T, Vector3dVector& points) {
  std::transform(points.begin(), points.end(), points.begin(), [&](const auto& point) { return T * point; });
}

inline Eigen::Vector3d compute_point_to_point_residual(const Sophus::SE3d& pose,
                                                       const OneCorrespondence& correspondence) {
  const auto& [source, target] = correspondence;
  const Eigen::Vector3d residual = (pose * source) - target;
  return residual;
}

inline Eigen::Vector3d compute_acceleration_cost_residual(const Eigen::Vector3d& local_gravity_estimate,
                                                          const Sophus::SO3d& current_rotation) {
  const Eigen::Vector3d predicted_gravity =
      current_rotation.inverse() * (-1 * gravity()); // points upwards, same as local_gravity_estimate
  const Eigen::Vector3d error = predicted_gravity - local_gravity_estimate;
  return error;
}

using LinearSystem = std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double>;
LinearSystem build_icp_linear_system(const Sophus::SE3d& current_pose, const Correspondences& correspondences) {
  auto linear_system_reduce = [](LinearSystem lhs, const LinearSystem& rhs) {
    auto& [lhs_H, lhs_b, lhs_chi] = lhs;
    const auto& [rhs_H, rhs_b, rhs_chi] = rhs;
    lhs_H += rhs_H;
    lhs_b += rhs_b;
    lhs_chi += rhs_chi;
    return lhs;
  };

  auto calculate_icp_jacobian = [](const Sophus::SE3d& current_pose, const OneCorrespondence& correspondence) {
    const auto& [source, _] = correspondence;
    Eigen::Matrix3_6d J_icp_l = Eigen::Matrix3_6d::Zero();
    J_icp_l.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    J_icp_l.block<3, 3>(0, 3) = -1 * Sophus::SO3d::hat(current_pose * source);
    return J_icp_l;
  };

  const auto& [H_icp, b_icp, chi_icp] =
      std::transform_reduce(correspondences.cbegin(), correspondences.cend(),
                            LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero(), 0.0), linear_system_reduce,
                            // transform
                            [&](const auto& correspondence) {
                              const Eigen::Vector3d residual =
                                  compute_point_to_point_residual(current_pose, correspondence);
                              const auto J = calculate_icp_jacobian(current_pose, correspondence);
                              return LinearSystem(J.transpose() * J,        // JT * R.inv() * J
                                                  J.transpose() * residual, // JT * R.inv() * r
                                                  residual.squaredNorm());  // chi
                            });

  return {H_icp / correspondences.size(), b_icp / correspondences.size(), 0.5 * chi_icp};
}

LinearSystem build_orientation_linear_system(const Sophus::SE3d& current_pose,
                                             const Eigen::Vector3d& local_gravity_estimate) {
  auto calculate_acceleration_jacobian = [](const Sophus::SO3d& current_rotation) {
    Eigen::Matrix3_6d J_ori = Eigen::Matrix3_6d::Zero();
    J_ori.block<3, 3>(0, 3) = current_rotation.inverse().matrix() * Sophus::SO3d::hat(-1 * gravity()).matrix();
    return J_ori;
  };

  const auto& [H_ori, b_ori, chi_ori] = std::invoke([&]() {
    const Eigen::Vector3d residual = compute_acceleration_cost_residual(local_gravity_estimate, current_pose.so3());
    const Eigen::Matrix3_6d J_ori = calculate_acceleration_jacobian(current_pose.so3());
    return LinearSystem{J_ori.transpose() * J_ori, J_ori.transpose() * residual, residual.squaredNorm()};
  });

  return {H_ori, b_ori, 0.5 * chi_ori};
}

Sophus::SE3d icp(const Vector3dVector& frame,
                 const SparseVoxelGrid& voxel_map,
                 const Sophus::SE3d& initial_guess,
                 const LIO::Config& config,
                 const std::optional<AccelInfo>& optional_accel_info) {
  // in case config disables it, or we don't have valid IMU information for this icp loop, beta is -1
  const double beta = (config.min_beta > 0 && optional_accel_info.has_value())
                          ? (config.min_beta * (1 + optional_accel_info->accel_mag_variance))
                          : -1;

  Sophus::SE3d current_pose = initial_guess;

  for (size_t i = 0; i < config.max_iterations; ++i) {

    const Correspondences& correspondences = data_association(current_pose, frame, voxel_map, config);
    if (correspondences.empty()) {
      throw std::runtime_error("Number of correspondences are 0.");
    }

    const auto& [H, b, chi] = std::invoke([&]() -> LinearSystem {
      if (beta >= 0) {
        const auto& [H_icp, b_icp, chi_icp] = build_icp_linear_system(current_pose, correspondences);
        const auto& [H_ori, b_ori, chi_ori] =
            build_orientation_linear_system(current_pose, optional_accel_info->local_gravity_estimate);
        return {H_icp + H_ori / beta, b_icp + b_ori / beta, chi_icp + chi_ori / beta};
      } else {
        return build_icp_linear_system(current_pose, correspondences);
      }
    });

    const Eigen::Vector6d dx = H.ldlt().solve(-b);
    current_pose = Sophus::SE3d::exp(dx) * current_pose;

    if (dx.norm() < config.convergence_criterion || i == (config.max_iterations - 1)) {
      // TODO: proper debug logging
      // std::cout << "iter " << i << ", beta: " << beta << ", chi: " << chi << ", num_assoc: " <<
      // correspondences.size() << "\n";
      break;
    }
  }
  return current_pose;
}

inline Sophus::SO3d align_accel_to_z_world(const Eigen::Vector3d& accel) {
  //  unobservable in the gravity direction, and the z in R.log() will always be 0
  const Eigen::Vector3d z_world = {0.0, 0.0, 1.0};
  const Eigen::Quaterniond quat_accel = Eigen::Quaterniond::FromTwoVectors(accel, z_world);
  return Sophus::SO3d(quat_accel);
}
} // namespace

// ==========================
//   actual LIO class stuff
// ==========================

namespace rko_lio::core {

// ==========================
//          private
// ==========================

void LIO::initialize(const Secondsd lidar_time) {
  if (interval_stats.imu_count == 0) {
    std::cerr << "[WARNING] Cannot initialize. No imu measurements received.\n";
    _initialized = true;
    return;
  }

  const Eigen::Vector3d avg_accel = interval_stats.imu_acceleration_sum / interval_stats.imu_count;
  const Eigen::Vector3d avg_gyro = interval_stats.angular_velocity_sum / interval_stats.imu_count;

  _imu_local_rotation = align_accel_to_z_world(avg_accel);
  _imu_local_rotation_time = lidar_time;
  lidar_state.pose.so3() = _imu_local_rotation;
  lidar_state.time = lidar_time;

  const Eigen::Vector3d local_gravity = _imu_local_rotation.inverse() * gravity();
  imu_bias.accelerometer = avg_accel + local_gravity;
  imu_bias.gyroscope = avg_gyro;

  _initialized = true;
  std::cout << "LIO initialized using " << interval_stats.imu_count
            << " IMU measurements. Estimated starting rotation [se(3)] is " << _imu_local_rotation.log().transpose()
            << ". Estimated accel bias: " << imu_bias.accelerometer.transpose()
            << ", gyro bias: " << imu_bias.gyroscope.transpose() << "\n";
}

// use the acceleration kalman filter to compute the two values we need for ori. reg.
std::optional<AccelInfo> LIO::get_accel_info(const Sophus::SO3d& rotation_estimate, const Secondsd& time) {
  if (interval_stats.imu_count <= 1) {
    std::cerr << "[WARNING] " << interval_stats.imu_count
              << " IMU message(s) in interval between two lidar scans. Cannot compute "
                 "acceleration statistics for orientation regularisation. Please check your data and its "
                 "timestamping as likely there should not be so few IMU measurements between two LiDAR scans.\n";
    return std::nullopt;
  }

  const Eigen::Vector3d avg_imu_accel = interval_stats.imu_acceleration_sum / interval_stats.imu_count;
  const double accel_mag_variance = interval_stats.welford_sum_of_squares / (interval_stats.imu_count - 1);
  const double dt = (time - lidar_state.time).count();

  const Eigen::Vector3d& body_accel_measurement = avg_imu_accel + rotation_estimate.inverse() * gravity();

  const double max_acceleration_change = config.max_expected_jerk * dt;
  // assume [j, -j] range for uniform dist. on jerk. variance is (2j)^2 / 12 = j^2/3. multiply by dt^2 for accel
  const Eigen::Matrix3d process_noise = square(max_acceleration_change) / 3 * Eigen::Matrix3d::Identity();
  body_acceleration_covariance += process_noise;

  // isotropic accel mag variance
  const Eigen::Matrix3d measurement_noise = accel_mag_variance / 3 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d S = body_acceleration_covariance + measurement_noise;
  const Eigen::Matrix3d kalman_gain = body_acceleration_covariance * S.inverse();

  const Eigen::Vector3d innovation = kalman_gain * (body_accel_measurement - mean_body_acceleration);
  mean_body_acceleration += innovation;
  body_acceleration_covariance -= kalman_gain * body_acceleration_covariance;

  const Eigen::Vector3d local_gravity_estimate = avg_imu_accel - mean_body_acceleration; // points upwards

  return AccelInfo{.accel_mag_variance = accel_mag_variance, .local_gravity_estimate = local_gravity_estimate};
}

// ==========================
//          public
// ==========================

// ============================ imu ===============================

void LIO::add_imu_measurement(const ImuControl& base_imu) {
  if (lidar_state.time < EPSILON_TIME) {
    static bool warning_skip_till_first_lidar = false;
    if (!warning_skip_till_first_lidar) {
      std::cerr << "[WARNING - ONCE] Skipping IMU, waiting for first LiDAR message.\n";
      warning_skip_till_first_lidar = true;
    }
    _last_real_imu_time = base_imu.time;
    _last_real_base_imu_ang_vel = base_imu.angular_velocity;
    return;
  }

  if (_imu_local_rotation_time < EPSILON_TIME) {
    _imu_local_rotation_time = lidar_state.time;
  }

  const double dt = (base_imu.time - _imu_local_rotation_time).count();

  if (dt < 0.0) {
    // messages are out of sync. thats a problem, since we integrate gyro from last lidar time onwards
    std::cerr << "[WARNING] Received IMU message from the past. Can result in errors.\n";
    // skip this imu reading?
  }

  const Eigen::Vector3d unbiased_ang_vel = base_imu.angular_velocity - imu_bias.gyroscope;
  const Eigen::Vector3d unbiased_accel = base_imu.acceleration - imu_bias.accelerometer;

  _imu_local_rotation = _imu_local_rotation * Sophus::SO3d::exp(unbiased_ang_vel * dt);
  _imu_local_rotation_time = base_imu.time;

  const Eigen::Vector3d local_gravity = _imu_local_rotation.inverse() * gravity();
  const Eigen::Vector3d compensated_accel = unbiased_accel + local_gravity;

  interval_stats.update(unbiased_ang_vel, unbiased_accel, compensated_accel);

  _last_real_imu_time = base_imu.time;
  _last_real_base_imu_ang_vel = base_imu.angular_velocity;
}

void LIO::add_imu_measurement(const Sophus::SE3d& extrinsic_imu2base, const ImuControl& raw_imu) {
  if (extrinsic_imu2base.log().norm() < EPSILON) {
    add_imu_measurement(raw_imu);
    return;
  }

  if (_last_real_imu_time < EPSILON_TIME) {
    // skip IMU message as we need a previous imu time for extrinsic compensation
    _last_real_imu_time = raw_imu.time;
    return;
  }

  // accounting for the transport-rate
  ImuControl base_imu = raw_imu;
  const Sophus::SO3d& extrinsic_rotation = extrinsic_imu2base.so3();
  base_imu.angular_velocity = extrinsic_rotation * raw_imu.angular_velocity;

  const Eigen::Vector3d& lever_arm = -1 * extrinsic_imu2base.translation();
  const Secondsd dt = raw_imu.time - _last_real_imu_time;

  const Eigen::Vector3d angular_acceleration = std::invoke([&]() -> Eigen::Vector3d {
    if (std::chrono::abs(dt) < Secondsd(1.0 / 5000.0)) {
      // if dt is less than the equivalent of a 5000 Hz imu, assuming zero ang accel,
      // causes numerical issues otherwise
      static bool warning_imu_too_close = false;
      if (!warning_imu_too_close) {
        std::cerr << "[WARNING - ONCE] Received IMU message with a very short delta to previous IMU message. Ignoring "
                     "all such messages.\n";
        warning_imu_too_close = true;
      }
      return Eigen::Vector3d::Zero();
    } else {
      const Eigen::Vector3d angular_acceleration =
          (base_imu.angular_velocity - _last_real_base_imu_ang_vel) / dt.count();
      return angular_acceleration;
    }
  });

  base_imu.acceleration = extrinsic_rotation * raw_imu.acceleration + angular_acceleration.cross(lever_arm) +
                          base_imu.angular_velocity.cross(base_imu.angular_velocity.cross(lever_arm));

  this->add_imu_measurement(base_imu);
}

// ============================ lidar ===============================

Vector3dVector LIO::register_scan(const Vector3dVector& scan, const TimestampVector& timestamps) {
  // TODO: redundant max compute as its available after process_timestamps. but changing the API of register scan
  // requires significant modifications and a whole lot of new boilerplate for the pybind. i'm leaving this alone until
  // i'm able to think of a better design
  const auto max = std::max_element(timestamps.cbegin(), timestamps.cend());
  const Secondsd current_lidar_time = *max;

  if (lidar_state.time < EPSILON_TIME) {
    lidar_state.time = current_lidar_time;
    std::cout << "First LiDAR received, using pose at this time as the global frame.\n";
    const auto& preproc_result = preprocess_scan(scan, config);
    if (!config.initialization_phase) {
      // use the first frame for the map only if we're not initializing
      map.Update(preproc_result.map_update_frame(), lidar_state.pose);
    }
    poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);
    return preproc_result.filtered_frame;
  }

  if (std::chrono::abs(current_lidar_time - lidar_state.time).count() > 1.0) {
    const double diff_seconds = (current_lidar_time - lidar_state.time).count();
    // TODO: std::expected with tl::expected (because ros humble)
    throw std::invalid_argument("Received LiDAR scan with " + std::to_string(diff_seconds) +
                                " seconds delta to previous scan.");
  }

  const auto& [avg_body_accel, avg_ang_vel] = std::invoke([&]() -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
    if (config.initialization_phase && !_initialized) {
      // assume static and
      initialize(current_lidar_time);
      return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }
    if (interval_stats.imu_count == 0) {
      std::cerr << "[WARNING] No Imu measurements in interval to average. Assuming constant velocity motion.\n";
      return {Eigen::Vector3d::Zero(), lidar_state.angular_velocity};
    }
    const Eigen::Vector3d avg_body_accel = interval_stats.body_acceleration_sum / interval_stats.imu_count;
    const Eigen::Vector3d avg_ang_vel = interval_stats.angular_velocity_sum / interval_stats.imu_count;
    if (avg_body_accel.norm() > 50.0) {
      std::cerr << "[WARNING] Erratic body acceleration computed, norm > 50 m/s2. Either IMU data is corrupted, or you "
                   "should report an issue.";
    }
    return {avg_body_accel, avg_ang_vel};
  });

  // compute relative motion using controls
  auto relative_pose_at_time = [&](const Secondsd time) -> Sophus::SE3d {
    const double dt = (time - lidar_state.time).count();
    Eigen::Matrix<double, 6, 1> tau;
    tau.head<3>() = lidar_state.velocity * dt + (avg_body_accel * square(dt) / 2);
    tau.tail<3>() = avg_ang_vel * dt;
    return Sophus::SE3d::exp(tau);
  };

  const Sophus::SE3d initial_guess = lidar_state.pose * relative_pose_at_time(current_lidar_time);

  // body acceleration filter
  const auto& accel_filter_info = get_accel_info(initial_guess.so3(), current_lidar_time);

  const auto& preproc_result = preprocess_scan(scan, timestamps, current_lidar_time, relative_pose_at_time, config);

  if (!map.Empty()) {
    SCOPED_PROFILER("ICP");
    const Sophus::SE3d optimized_pose = icp(preproc_result.keypoints, map, initial_guess, config, accel_filter_info);

    // estimate velocities and accelerations from the new pose
    const double dt = (current_lidar_time - lidar_state.time).count();
    const Sophus::SE3d motion = lidar_state.pose.inverse() * optimized_pose;
    const Eigen::Vector6d local_velocity = motion.log() / dt;
    const Eigen::Vector3d local_linear_acceleration =
        (local_velocity.head<3>() - motion.so3().inverse() * lidar_state.velocity) / dt;

    // update
    lidar_state.pose = optimized_pose;
    lidar_state.velocity = local_velocity.head<3>();
    lidar_state.angular_velocity = local_velocity.tail<3>();
    lidar_state.linear_acceleration = local_linear_acceleration;

    _imu_local_rotation = optimized_pose.so3(); // correct the drift in imu integration
  }
  // even if map is empty, time should still update
  lidar_state.time = current_lidar_time;
  _imu_local_rotation_time = current_lidar_time;

  // reset imu averages
  interval_stats.reset();

  map.Update(preproc_result.map_update_frame(), lidar_state.pose);

  poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);

  return preproc_result.filtered_frame;
}

Vector3dVector LIO::register_scan(const Sophus::SE3d& extrinsic_lidar2base,
                                  const Vector3dVector& scan,
                                  const TimestampVector& timestamps) {
  if (extrinsic_lidar2base.log().norm() < EPSILON) {
    return register_scan(scan, timestamps);
  }

  Vector3dVector transformed_scan = scan;
  transform_points(extrinsic_lidar2base, transformed_scan);
  Vector3dVector frame = register_scan(transformed_scan, timestamps);
  transform_points(extrinsic_lidar2base.inverse(), frame);
  return frame;
}
} // namespace rko_lio::core
