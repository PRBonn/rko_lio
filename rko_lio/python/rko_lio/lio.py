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

import numpy as np

from .config import LIOConfig
from .rko_lio_pybind import (
    _LIO,
    _IntervalStats,
    _Vector3dVector,
    _VectorDouble,
)


class IntervalStats:
    """Convenience class to compute the interval actual averages"""

    def __init__(self, raw_stats: _IntervalStats):
        self._impl = raw_stats

    @property
    def imu_count(self):
        return self._impl.imu_count

    @property
    def imu_accel_mag_mean(self):
        return self._impl.imu_accel_mag_mean

    @property
    def welford_sum_of_squares(self):
        return self._impl.welford_sum_of_squares

    @property
    def imu_acceleration_sum(self):
        return np.array(self._impl.imu_acceleration_sum)

    @property
    def body_acceleration_sum(self):
        return np.array(self._impl.body_acceleration_sum)

    @property
    def angular_velocity_sum(self):
        return np.array(self._impl.angular_velocity_sum)

    def __repr__(self):
        return (
            f"IntervalStats(imu_count={self.imu_count}, "
            f"imu_accel_mag_mean={self.imu_accel_mag_mean:.6f}, "
            f"welford_sum_of_squares={self.welford_sum_of_squares:.6f}, "
            f"imu_acceleration_sum={self.imu_acceleration_sum}, "
            f"body_acceleration_sum={self.body_acceleration_sum}, "
            f"angular_velocity_sum={self.angular_velocity_sum})"
        )

    def avg_imu_accel(self):
        if self.imu_count == 0:
            return np.zeros(3)
        return self.imu_acceleration_sum / self.imu_count

    def avg_body_accel(self):
        if self.imu_count == 0:
            return np.zeros(3)
        return self.body_acceleration_sum / self.imu_count

    def avg_ang_vel(self):
        if self.imu_count == 0:
            return np.zeros(3)
        return self.angular_velocity_sum / self.imu_count

    def accel_mag_variance(self):
        if self.imu_count < 2:
            return 0.0
        return self.welford_sum_of_squares / (self.imu_count - 1)

    def accel_mag_stddev(self):
        return np.sqrt(self.accel_mag_variance())


class LIO:
    def __init__(self, config: LIOConfig):
        self.config = config
        self._impl = _LIO(config)

    def __repr__(self):
        return f"LIO with config: {repr(self.config)}"

    def interval_stats(self):
        """Can be useful for introspecting some IMU details."""
        return IntervalStats(self._impl.interval_stats)

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

    def poses_with_timestamps(self):
        timestamps, poses = self._impl.poses_with_timestamps()
        return np.asarray(timestamps), poses
