#include "preprocess_scan.hpp"
#include "voxel_down_sample.hpp"

namespace rko_lio::core {

PreprocessingResult preprocess_scan(const Vector3dVector& frame, const LIO::Config& config) {
  std::vector<Eigen::Vector3d> clipped_frame;
  clipped_frame.reserve(frame.size());

  std::for_each(frame.cbegin(), frame.cend(), [&](const auto& point) {
    const double point_range = point.norm();
    if (point_range > config.min_range && point_range < config.max_range) {
      clipped_frame.emplace_back(point);
    }
  });
  clipped_frame.shrink_to_fit();

  if (config.double_downsample) {
    const Vector3dVector downsampled_frame = voxel_down_sample(clipped_frame, config.voxel_size * 0.5);
    const Vector3dVector keypoints = voxel_down_sample(downsampled_frame, config.voxel_size * 1.5);
    return {.filtered_frame = clipped_frame, .map_frame = downsampled_frame, .keypoints = keypoints};
  } else {
    const Vector3dVector downsampled_frame = voxel_down_sample(clipped_frame, config.voxel_size);
    return {.filtered_frame = clipped_frame, .map_frame = std::nullopt, .keypoints = downsampled_frame};
  }
}

} // namespace rko_lio::core
