#pragma once
#include <Eigen/Core>
#include <vector>

namespace rko_lio::tests {

// Hollow cube of side 2*half_extent centered at the origin, six faces sampled at
// `spacing`.
inline std::vector<Eigen::Vector3s> make_hollow_cube(double spacing = 0.5, double half_extent = 5.0) {
  std::vector<Eigen::Vector3s> points;
  const int n = static_cast<int>(2 * half_extent / spacing) + 1;
  points.reserve(static_cast<size_t>(6 * n * n));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double a = -half_extent + i * spacing;
      const double b = -half_extent + j * spacing;
      points.emplace_back(half_extent, a, b);
      points.emplace_back(-half_extent, a, b);
      points.emplace_back(a, half_extent, b);
      points.emplace_back(a, -half_extent, b);
      points.emplace_back(a, b, half_extent);
      points.emplace_back(a, b, -half_extent);
    }
  }
  return points;
}

} // namespace rko_lio::tests
