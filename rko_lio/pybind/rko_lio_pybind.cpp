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

#include "rko_lio/core/lio.hpp"
#include "rko_lio/core/process_timestamps.hpp"
#include "stl_vector_eigen.hpp"
#include <cmath>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace rko_lio::core;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);

PYBIND11_MODULE(rko_lio_pybind, m) {
  auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
      m, "_Vector3dVector", "std::vector<Eigen::Vector3d>", py::py_array_to_vectors_double<Eigen::Vector3d>);
  py::bind_vector<std::vector<double>>(m, "_VectorDouble");

  using Config = LIO::Config;
  py::class_<Config>(m, "_Config")
      .def(py::init<>())
      .def_readwrite("deskew", &Config::deskew)
      .def_readwrite("max_iterations", &Config::max_iterations)
      .def_readwrite("voxel_size", &Config::voxel_size)
      .def_readwrite("max_points_per_voxel", &Config::max_points_per_voxel)
      .def_readwrite("max_range", &Config::max_range)
      .def_readwrite("min_range", &Config::min_range)
      .def_readwrite("convergence_criterion", &Config::convergence_criterion)
      .def_readwrite("max_correspondance_distance", &Config::max_correspondance_distance)
      .def_readwrite("max_num_threads", &Config::max_num_threads)
      .def_readwrite("initialization_phase", &Config::initialization_phase)
      .def_readwrite("max_expected_jerk", &Config::max_expected_jerk)
      .def_readwrite("double_downsample", &Config::double_downsample)
      .def_readwrite("min_beta", &Config::min_beta);

  py::class_<LIO>(m, "_LIO")
      .def(py::init<const Config&>(), "config"_a)
      .def(
          "add_imu_measurement",
          [](LIO& self, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const double time) {
            self.add_imu_measurement(ImuControl{
                .time = Secondsd(time),
                .acceleration = accel,
                .angular_velocity = gyro,
            });
          },
          "acceleration"_a, "angular_velocity"_a, "time"_a)
      .def(
          "add_imu_measurement",
          [](LIO& self, const Eigen::Matrix4d& extrinsic_imu2base, const Eigen::Vector3d& accel,
             const Eigen::Vector3d& gyro, const double time) {
            self.add_imu_measurement(Sophus::SE3d(extrinsic_imu2base), ImuControl{
                                                                           .time = Secondsd(time),
                                                                           .acceleration = accel,
                                                                           .angular_velocity = gyro,
                                                                       });
          },
          "extrinsic_imu2base"_a, "acceleration"_a, "angular_velocity"_a, "time"_a)
      .def(
          "register_scan",
          [](LIO& self, const std::vector<Eigen::Vector3d>& scan, const std::vector<double>& timestamps) {
            std::vector<Secondsd> tsd(timestamps.size());
            std::transform(timestamps.cbegin(), timestamps.cend(), tsd.begin(),
                           [](const double t) { return Secondsd(t); });
            return self.register_scan(scan, tsd);
          },
          "scan"_a, "timestamps"_a)
      .def(
          "register_scan",
          [](LIO& self, const Eigen::Matrix4d& extrinsic_lidar2base, const std::vector<Eigen::Vector3d>& scan,
             const std::vector<double>& timestamps) {
            std::vector<Secondsd> tsd(timestamps.size());
            std::transform(timestamps.cbegin(), timestamps.cend(), tsd.begin(),
                           [](const double t) { return Secondsd(t); });
            return self.register_scan(Sophus::SE3d(extrinsic_lidar2base), scan, tsd);
          },
          "extrinsic_lidar2base"_a, "scan"_a, "timestamps"_a)
      .def("map_point_cloud", [](LIO& self) { return self.map.Pointcloud(); })
      .def("pose", [](LIO& self) { return self.lidar_state.pose.matrix(); })
      .def("poses_with_timestamps", [](LIO& self) {
        const size_t n = self.poses_with_timestamps.size();
        std::vector<double> times(n);
        pybind11::array_t<double> poses({n, static_cast<size_t>(7)});
        // https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html#direct-access
        auto pose_buf = poses.mutable_unchecked<2>();
        for (size_t i = 0; i < n; ++i) {
          const auto& [time, pose] = self.poses_with_timestamps[i];
          times[i] = time.count();
          const Eigen::Vector3d& trans = pose.translation();
          const Eigen::Quaterniond& q = pose.unit_quaternion();
          pose_buf(i, 0) = trans.x();
          pose_buf(i, 1) = trans.y();
          pose_buf(i, 2) = trans.z();
          pose_buf(i, 3) = q.x();
          pose_buf(i, 4) = q.y();
          pose_buf(i, 5) = q.z();
          pose_buf(i, 6) = q.w();
        }
        return pybind11::make_tuple(times, poses);
      });

  m.def(
      "_process_timestamps",
      [](const std::vector<double>& raw_ts, const double header_stamp) {
        const auto& [begin, end, abs_ts] = process_timestamps(raw_ts, Secondsd(header_stamp));
        std::vector<double> abs_ts_double(abs_ts.size());
        std::transform(abs_ts.cbegin(), abs_ts.cend(), abs_ts_double.begin(), [&](const auto& t) { return t.count(); });
        return std::make_tuple(begin.count(), end.count(), std::move(abs_ts_double));
      },
      "raw_timestamps"_a, "header_stamp"_a);
}
