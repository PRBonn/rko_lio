#pragma once
#include <Eigen/Dense>
#include <vector>

#include "lio.hpp"

namespace rko_lio::core {

struct PreprocessingResult {
  Vector3sVector filtered_scan;
  Vector3sVector keypoints;
  Vector3sVector map_points; // populated only when config.double_downsample; empty otherwise
};

// clip and downsample the input cloud
PreprocessingResult preprocess_scan(Vector3sVector scan, const LIO::Config& config);

} // namespace rko_lio::core
