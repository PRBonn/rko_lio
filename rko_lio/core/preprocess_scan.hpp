#pragma once
#include <Eigen/Dense>
#include <optional>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

#include "lio.hpp"

namespace rko_lio::core {

struct PreprocessingResult {
  Vector3dVector filtered_frame;
  std::optional<Vector3dVector> map_frame;
  Vector3dVector keypoints;

  const Vector3dVector& map_update_frame() const { return map_frame ? *map_frame : keypoints; }
};

// clip and downsample the input cloud
PreprocessingResult preprocess_scan(const Vector3dVector& frame, const LIO::Config& config);

template <typename Functor>
  requires requires(Functor f, Secondsd stamp) {
    { f(stamp) } -> std::same_as<Sophus::SE3d>;
  }
PreprocessingResult preprocess_scan(const Vector3dVector& frame,
                                    const TimestampVector& timestamps,
                                    Secondsd end_time,
                                    const Functor& relative_pose_at_time,
                                    const LIO::Config config) {
  if (!config.deskew) {
    return preprocess_scan(frame, config);
  }

  const Sophus::SE3d scan_to_scan_motion_inverse = relative_pose_at_time(end_time).inverse();
  Vector3dVector deskewed_frame(frame.size());
  std::transform(frame.cbegin(), frame.cend(), timestamps.cbegin(), deskewed_frame.begin(),
                 [&](const Eigen::Vector3d& point, Secondsd timestamp) {
                   const auto pose = scan_to_scan_motion_inverse * relative_pose_at_time(timestamp);
                   return pose * point;
                 });

  return preprocess_scan(deskewed_frame, config);
}

} // namespace rko_lio::core
