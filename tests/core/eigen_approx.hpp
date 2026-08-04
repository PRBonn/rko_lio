#pragma once
#include <rko_lio/core/util.hpp>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <type_traits>

namespace rko_lio::tests {

constexpr bool IS_DOUBLE = std::is_same_v<core::Scalar, double>;

// double has tighter tolerances when being used. float is looser
constexpr double LOOSE_TOL = IS_DOUBLE ? 1e-6 : 1e-4;
constexpr double TOL = IS_DOUBLE ? 1e-9 : 1e-4;
constexpr double EXACT_TOL = IS_DOUBLE ? 1e-12 : 1e-4;

inline bool approx_equal(const Eigen::Vector3s& a, const Eigen::Vector3s& b, double tolerance = LOOSE_TOL) {
  return (a - b).norm() < tolerance;
}

inline bool approx_equal(const Sophus::SO3s& a, const Sophus::SO3s& b, double tolerance = LOOSE_TOL) {
  return (a.inverse() * b).log().norm() < tolerance;
}

inline bool approx_equal(const Sophus::SE3s& a, const Sophus::SE3s& b, double tolerance = LOOSE_TOL) {
  return (a.inverse() * b).log().norm() < tolerance;
}

} // namespace rko_lio::tests
