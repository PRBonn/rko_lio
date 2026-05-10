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

import atexit
import threading
from pathlib import Path

import numpy as np
import yaml

from .config import PipelineConfig
from .lio import LIO
from .scoped_profiler import ScopedProfiler, profile_func
from .util import (
    height_colors_from_points,
    info,
    log_vector,
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
    ):
        self.config = config
        self.map_log_period_s = map_log_period_s
        self.lio = LIO(config.lio)
        self.extrinsic_imu2base = quat_xyzw_xyz_to_transform(
            config.extrinsic_imu2base_quat_xyzw_xyz
        )
        self.extrinsic_lidar2base = quat_xyzw_xyz_to_transform(
            config.extrinsic_lidar2base_quat_xyzw_xyz
        )

        self._output_dir = None

        if self.config.viz:
            import rerun

            self.rerun = rerun
            self.last_xyz = np.zeros(3)
            self.cloud_box = LatestMailbox()
            self.last_map_log_s = -float("inf")
            self.cloud_thread = threading.Thread(
                target=self.cloud_log_loop, daemon=True
            )
            self.cloud_thread.start()
            atexit.register(self.close)
            if self.lio.config.initialization_phase:
                self.rerun.log(
                    "world",
                    self.rerun.ViewCoordinates.RIGHT_HAND_Z_UP,
                    static=True,
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

        if self.config.viz:
            self.rerun.set_time("data_time", timestamp=time * 1e-9)
            log_vector(self.rerun, "imu/acceleration", acceleration)
            log_vector(self.rerun, "imu/angular_velocity", angular_velocity)

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
        if self.config.viz:
            # needs to be logged before the pybinded register function is called
            self.rerun.set_time("data_time", timestamp=end_time_ns * 1e-9)
            stats = self.lio.interval_stats()
            self.rerun.log("imu/imu_count", self.rerun.Scalars(float(stats.imu_count)))
            log_vector(self.rerun, "imu/avg_acceleration", stats.avg_imu_accel())
            log_vector(self.rerun, "imu/avg_body_acceleration", stats.avg_body_accel())
            log_vector(self.rerun, "imu/avg_ang_velocity", stats.avg_ang_vel())

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

        if self.config.viz:
            self.visualize(end_time_ns, deskewed_scan)

        return deskewed_scan

    @profile_func("Pipeline - Visualization")
    def visualize(self, end_time_ns: int, deskewed_scan: np.ndarray):
        scan_time_s = end_time_ns * 1e-9
        self.rerun.set_time("data_time", timestamp=scan_time_s)
        pose = self.lio.pose()  # base -> world
        self.rerun.log(
            "world/base",
            self.rerun.Transform3D(translation=pose[:3, 3], mat3x3=pose[:3, :3]),
        )
        self.rerun.log("world/base", self.rerun.TransformAxes3D(2.0))
        self.rerun.log(
            "world/view_anchor",
            self.rerun.Transform3D(translation=pose[:3, 3]),
        )
        traj_pts = np.array([self.last_xyz, pose[:3, 3]])
        self.rerun.log(
            "world/trajectory",
            self.rerun.LineStrips3D([traj_pts], radii=[0.1], colors=[255, 111, 111]),
        )
        self.last_xyz = pose[:3, 3].copy()

        # Hand the heavy work off to the worker thread
        # Local map is sampled here as the next register_scan mutates it which can race
        T_lidar2world = pose @ self.extrinsic_lidar2base
        local_map = None
        if scan_time_s - self.last_map_log_s >= self.map_log_period_s:
            pts = self.lio.map_point_cloud()
            if pts.size > 0:
                local_map = pts
                self.last_map_log_s = scan_time_s

        self.cloud_box.put((scan_time_s, deskewed_scan, T_lidar2world, local_map))

    def cloud_log_loop(self):
        # send_columns binds the timestamp to the data, so we don't race the
        # main thread's rr.set_time() global timeline state.
        rr = self.rerun
        for item in iter(self.cloud_box.get, None):
            scan_time_s, deskewed_scan, T_lidar2world, local_map = item
            scan_world = (T_lidar2world[:3, :3] @ deskewed_scan.T).T + T_lidar2world[
                :3, 3
            ]
            time_idx = [rr.TimeColumn("data_time", timestamp=[scan_time_s])]
            rr.send_columns(
                "world/deskewed_scan",
                indexes=time_idx,
                columns=rr.Points3D.columns(positions=scan_world).partition(
                    [len(scan_world)]
                ),
            )
            if local_map is not None:
                rr.send_columns(
                    "world/local_map",
                    indexes=time_idx,
                    columns=rr.Points3D.columns(
                        positions=local_map,
                        colors=height_colors_from_points(local_map),
                    ).partition([len(local_map)]),
                )

    def close(self):
        if not self.config.viz:
            return
        self.cloud_box.close()
        self.cloud_thread.join()

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


class LatestMailbox:
    """
    Single-slot, latest-wins handoff between producer and one consumer.
    Producer never blocks.
    """

    def __init__(self):
        self.cv = threading.Condition()
        self.item = None
        self.closed = False

    def put(self, item):
        with self.cv:
            self.item = item
            self.cv.notify()

    def get(self):
        with self.cv:
            self.cv.wait_for(lambda: self.item is not None or self.closed)
            item, self.item = self.item, None
            return item

    def close(self):
        with self.cv:
            self.closed = True
            self.cv.notify()
