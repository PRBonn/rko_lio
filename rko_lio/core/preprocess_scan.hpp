#pragma once
#include <Eigen/Dense>
#include <optional>
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

} // namespace rko_lio::core
