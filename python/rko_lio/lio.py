# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Public interface classes for the pybind.
"""

from pathlib import Path

import numpy as np

from .rko_lio_pybind import _LIO, _Config, _Vector3dVector, _VectorDouble


class LIOConfig(_Config):
    """
    LIO configuration options.

    Parameters
    ----------
    deskew : bool, default True
        If True, perform scan deskewing.
    max_iterations : int, default 100
        Maximum optimization iterations for scan matching.
    voxel_size : float, default 1.0
        Size of map voxels (meters).
    max_points_per_voxel : int, default 20
        Maximum points stored per voxel.
    max_range : float, default 100.0
        Max usable range of lidar (meters).
    min_range : float, default 1.0
        Minimum usable range of lidar (meters).
    convergence_criterion : float, default 1e-5
        Convergence threshold for optimization.
    max_correspondance_distance : float, default 0.5
        Max distance for associating points (meters).
    max_num_threads : int, default 0
        Max thread count (0 = autodetect).
    initialization_phase : bool, default False
        Whether to initialize on the first two lidar message.
    max_expected_jerk : float, default 3.0
        Max expected IMU jerk (m/s^3).
    double_downsample : bool, default True
        Double downsamples the incoming scan before registering. Disabling for sparse LiDARs may improve results.
    min_beta : float, default 200.0
        Minimum scaling on the orientation regularisation weight. Set to -1 to disable the cost.
    """

    def __init__(
        self,
        deskew: bool = True,
        max_iterations: int = 100,
        voxel_size: float = 1.0,
        max_points_per_voxel: int = 20,
        max_range: float = 100.0,
        min_range: float = 1.0,
        convergence_criterion: float = 1e-5,
        max_correspondance_distance: float = 0.5,
        max_num_threads: int = 0,
        initialization_phase: bool = False,
        max_expected_jerk: float = 3.0,
        double_downsample: bool = True,
        min_beta: float = 200.0,
    ):
        super().__init__()
        self.deskew = deskew
        self.max_iterations = max_iterations
        self.voxel_size = voxel_size
        self.max_points_per_voxel = max_points_per_voxel
        self.max_range = max_range
        self.min_range = min_range
        self.convergence_criterion = convergence_criterion
        self.max_correspondance_distance = max_correspondance_distance
        self.max_num_threads = max_num_threads
        self.initialization_phase = initialization_phase
        self.max_expected_jerk = max_expected_jerk
        self.double_downsample = double_downsample
        self.min_beta = min_beta

    def __repr__(self):
        return (
            f"LIOConfig("
            f"deskew={self.deskew}, "
            f"max_iterations={self.max_iterations}, "
            f"voxel_size={self.voxel_size}, "
            f"max_points_per_voxel={self.max_points_per_voxel}, "
            f"max_range={self.max_range}, "
            f"min_range={self.min_range}, "
            f"convergence_criterion={self.convergence_criterion}, "
            f"max_correspondance_distance={self.max_correspondance_distance}, "
            f"max_num_threads={self.max_num_threads}, "
            f"initialization_phase={self.initialization_phase}, "
            f"max_expected_jerk={self.max_expected_jerk}, "
            f"double_downsample={self.double_downsample}, "
            f"min_beta={self.min_beta})"
        )


class LIO:
    def __init__(self, config: LIOConfig):
        self.config = config
        self._impl = _LIO(config)

    def __repr__(self):
        return f"LIO with config: {repr(self.config)}"

    def map_point_cloud(self) -> np.ndarray:
        """return the local map point cloud *in world/odometry frame*"""
        return np.asarray(self._impl.map_point_cloud())

    def pose(self) -> np.ndarray:
        """return the 4x4 transform from the base frame to the world/odometry frame"""
        return np.asarray(self._impl.pose())

    def add_imu_measurement(
        self,
        acceleration: np.ndarray,
        angular_velocity: np.ndarray,
        time: float,
    ):
        acc = np.asarray(acceleration, dtype=np.float64)
        gyro = np.asarray(angular_velocity, dtype=np.float64)
        if acc.shape != (3,):
            raise ValueError(f"acceleration: expected shape (3,), got {acc.shape}")
        if gyro.shape != (3,):
            raise ValueError(f"angular_velocity: expected shape (3,), got {gyro.shape}")
        self._impl.add_imu_measurement(acc, gyro, float(time))

    def add_imu_measurement_with_extrinsic(
        self,
        extrinsic_imu2base: np.ndarray,
        acceleration: np.ndarray,
        angular_velocity: np.ndarray,
        time: float,
    ):
        extr = np.asarray(extrinsic_imu2base, dtype=np.float64)
        acc = np.asarray(acceleration, dtype=np.float64)
        gyro = np.asarray(angular_velocity, dtype=np.float64)
        if extr.shape != (4, 4):
            raise ValueError(
                f"extrinsic_imu2base: expected shape (4,4), got {extr.shape}"
            )
        if acc.shape != (3,):
            raise ValueError(f"acceleration: expected shape (3,), got {acc.shape}")
        if gyro.shape != (3,):
            raise ValueError(f"angular_velocity: expected shape (3,), got {gyro.shape}")
        self._impl.add_imu_measurement(extr, acc, gyro, float(time))

    def register_scan(self, scan: np.ndarray, timestamps: np.ndarray):
        scan_arr = np.asarray(scan, dtype=np.float64)
        times_arr = np.asarray(timestamps, dtype=np.float64)
        if scan_arr.ndim != 2 or scan_arr.shape[1] != 3:
            raise ValueError(f"scan: expected (N,3), got {scan_arr.shape}")
        if times_arr.shape != (scan_arr.shape[0],):
            raise ValueError(
                f"timestamps: expected ({scan_arr.shape[0]},), got {times_arr.shape}"
            )
        scan_vec = _Vector3dVector(scan_arr)
        time_vec = _VectorDouble(times_arr)
        ret_scan = self._impl.register_scan(scan_vec, time_vec)
        return np.asarray(ret_scan)

    def register_scan_with_extrinsic(
        self, extrinsic_lidar2base: np.ndarray, scan: np.ndarray, timestamps: np.ndarray
    ):
        extr = np.asarray(extrinsic_lidar2base, dtype=np.float64)
        scan_arr = np.asarray(scan, dtype=np.float64)
        times_arr = np.asarray(timestamps, dtype=np.float64)
        if extr.shape != (4, 4):
            raise ValueError(
                f"extrinsic_lidar2base: expected shape (4,4), got {extr.shape}"
            )
        if scan_arr.ndim != 2 or scan_arr.shape[1] != 3:
            raise ValueError(f"scan: expected (N,3), got {scan_arr.shape}")
        if times_arr.shape != (scan_arr.shape[0],):
            raise ValueError(
                f"timestamps: expected ({scan_arr.shape[0]},), got {times_arr.shape}"
            )
        scan_vec = _Vector3dVector(scan_arr)
        time_vec = _VectorDouble(times_arr)
        ret_scan = self._impl.register_scan(extr, scan_vec, time_vec)
        return np.asarray(ret_scan)

    def dump_results_to_disk(self, results_dir: Path, run_name: str):
        self._impl.dump_results_to_disk(str(results_dir), str(run_name))
