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
Equivalent logic to the ros node but mostly synchronous. Only the rerun viz is multi-threaded.
"""

from pathlib import Path

import numpy as np
import yaml

from .config import PipelineConfig
from .lio import LIO
from .scoped_profiler import profile_func
from .util import (
    info,
    quat_xyzw_xyz_to_transform,
    save_scan_as_ply,
)


class LIOPipeline:
    """
    Minimal sequential pipeline for LIO processing.
    """

    def __init__(
        self,
        config: PipelineConfig,
        map_log_period_s: float = 1.0,
        rbl_path: Path | None = None,
        reset_viz: bool = True,
    ):
        self.config = config
        self.lio = LIO(config.lio)
        self.extrinsic_imu2base = quat_xyzw_xyz_to_transform(
            config.extrinsic_imu2base_quat_xyzw_xyz
        )
        self.extrinsic_lidar2base = quat_xyzw_xyz_to_transform(
            config.extrinsic_lidar2base_quat_xyzw_xyz
        )

        self._output_dir = None
        self.viz = None

        if self.config.viz:
            from .rerun_viz import Viz

            self.viz = Viz(
                map_log_period_s=map_log_period_s,
                rbl_path=rbl_path,
                reset=reset_viz,
                gravity_aligned=self.lio.config.initialization_phase,
            )

    @property
    def output_dir(self) -> Path:
        """
        The directory used for file logging if enabled.
        Folder is {log_dir}/{run_name}_{index}.
        Automatically bumps the index (from 0) if similar names exist, to avoid overwriting.
        """
        if self._output_dir is not None:
            return self._output_dir

        self.config.log_dir.mkdir(parents=True, exist_ok=True)
        index = 0
        while True:
            output_dir = self.config.log_dir / f"{self.config.run_name}_{index}"
            if not output_dir.exists():
                break
            index += 1
        output_dir.mkdir()
        self._output_dir = output_dir
        return self._output_dir

    def add_imu(
        self,
        time: int,
        acceleration: np.ndarray,
        angular_velocity: np.ndarray,
    ):
        """
        Add IMU measurement to pipeline (will be buffered until processed by lidar).

        Parameters
        ----------
        time : int
            Measurement timestamp in nanoseconds (absolute, since the unix epoch).
        acceleration : array of float, shape (3,)
            Acceleration vector in m/s^2.
        angular_velocity : array of float, shape (3,)
            Angular velocity in rad/s.
        """
        self.lio.add_imu_measurement(
            acceleration=acceleration,
            angular_velocity=angular_velocity,
            time=time,
            extrinsic_imu2base=self.extrinsic_imu2base,
        )

        if self.viz:
            self.viz.log_imu(time, acceleration, angular_velocity)

    @profile_func("Pipeline - Register Scan")
    def register_scan(
        self,
        start_time_ns: int,
        end_time_ns: int,
        scan: np.ndarray,
        timestamps: np.ndarray,
    ):
        """
        Register a lidar scan.
        Timestamps are assumed to be absolute nanoseconds.
        It is assumed there is sufficient IMU data added to the pipeline before triggering the registration (use the Sequencer).


        Parameters
        ----------
        start_time_ns: int
            Absolute time of the scan recording start, in nanoseconds.
        end_time_ns: int
            Absolute time of the scan recording end, in nanoseconds.
        scan : array of float, shape (N,3)
            Point cloud.
        timestamps : array of int64, shape (N,)
            Absolute per-point timestamps in nanoseconds.

        Returns
        -------
        np.ndarray or None
            Deskewed scan if successful, None if registration failed
        """
        if self.viz:
            # needs to be logged before the pybinded register function is called
            self.viz.log_interval_stats(end_time_ns, self.lio.interval_stats())

        try:
            deskewed_scan = self.lio.register_scan(
                scan,
                timestamps,
                extrinsic_lidar2base=self.extrinsic_lidar2base,
            )
        except ValueError as e:
            print(
                "ERROR: Dropping LiDAR frame as there was an error. Odometry might suffer. Error:",
                e,
            )
            return None

        if self.config.dump_deskewed_scans:
            save_scan_as_ply(
                deskewed_scan,
                end_time_ns,
                output_dir=self.output_dir / "deskewed_scans",
            )

        if self.viz:
            # sampled now as the next register_scan mutates the map, which can race
            local_map = (
                self.lio.map_point_cloud()
                if self.viz.wants_local_map(end_time_ns)
                else None
            )
            self.viz.log_frame(
                end_time_ns,
                self.lio.pose(),
                deskewed_scan,
                self.extrinsic_lidar2base,
                local_map,
            )

        return deskewed_scan

    def close(self):
        if self.viz:
            self.viz.close()

    def dump_results_to_disk(self):
        """
        Write LIO results to disk under LIOPipeline.output_dir.

        Writes:
        - Trajectory (timestamps and poses) in TUM format text file.
        - Configuration as YAML file.
        """
        traj_file = self.output_dir / f"{self.output_dir.name}_tum.txt"
        timestamps_ns, poses = self.lio.poses_with_timestamps()
        with traj_file.open("w") as f:
            for t_ns, p in zip(timestamps_ns, poses):
                # p: x,y,z,qx,qy,qz,qw
                t_s = t_ns * 1e-9
                line = f"{t_s:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f} {p[4]:.6f} {p[5]:.6f} {p[6]:.6f}\n"
                f.write(line)
        info(f"Poses written to {traj_file.resolve()}")

        config = self.config.to_dict()
        settings_file = self.output_dir / "config.yaml"
        with settings_file.open("w") as f:
            yaml.dump(config, f, sort_keys=False)
        info(f"Configuration written to {settings_file.resolve()}")
