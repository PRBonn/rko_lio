#include "preprocess_scan.hpp"
#include "voxel_down_sample.hpp"

#include <cmath>

namespace rko_lio::core {

PreprocessingResult preprocess_scan(Vector3sVector scan, const LIO::Config& config) {
  std::erase_if(scan, [&](const auto& point) {
    const Scalar point_range = point.norm();
    return !std::isfinite(point_range) || point_range <= config.min_range || point_range >= config.max_range;
  });

  if (config.double_downsample) {
    Vector3sVector downsampled = voxel_down_sample(scan, config.voxel_size * static_cast<Scalar>(0.5));
    Vector3sVector keypoints = voxel_down_sample(downsampled, config.voxel_size * static_cast<Scalar>(1.5));
    return {.filtered_scan = std::move(scan), .keypoints = std::move(keypoints), .map_points = std::move(downsampled)};
  }
  Vector3sVector keypoints = voxel_down_sample(scan, config.voxel_size);
  return {.filtered_scan = std::move(scan), .keypoints = std::move(keypoints), .map_points = {}};
}

} // namespace rko_lio::core
