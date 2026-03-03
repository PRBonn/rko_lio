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
Equivalent logic to the ros wrapper's message buffering.
A convenience class to buffer IMU and LiDAR messages to ensure the core cpp implementation always
gets the data in sync.
The difference is this is not multi-threaded, therefore is a bit slower.
"""

from pathlib import Path

import numpy as np
import yaml

from .config import PipelineConfig
from .lio import LIO
from .scoped_profiler import ScopedProfiler
from .util import (
    height_colors_from_points,
    info,
    quat_xyzw_xyz_to_transform,
    save_scan_as_ply,
    transform_to_quat_xyzw_xyz,
)


class LIOPipeline:
    """
    Minimal sequential pipeline for LIO processing.
    """

    def __init__(
        self,
        config: PipelineConfig,
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

        if self.config.viz:
            import rerun

            self.rerun = rerun
            self.viz_counter = 0
            self.last_xyz = np.zeros(3)
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
        time: float,
        acceleration: np.ndarray,
        angular_velocity: np.ndarray,
    ):
        """
        Add IMU measurement to pipeline (will be buffered until processed by lidar).

        Parameters
        ----------
        time : float
            Measurement timestamp in seconds.
        acceleration : array of float, shape (3,)
            Acceleration vector in m/s^2.
        angular_velocity : array of float, shape (3,)
            Angular velocity in rad/s.
        """
        if self.extrinsic_imu2base is not None:
            self.lio.add_imu_measurement_with_extrinsic(
                self.extrinsic_imu2base,
                time=time,
                acceleration=acceleration,
                angular_velocity=angular_velocity,
            )
        else:
            self.lio.add_imu_measurement(
                time=time, acceleration=acceleration, angular_velocity=angular_velocity
            )

        if self.config.viz:
            self.rerun.set_time("data_time", timestamp=time)
            log_vector(self.rerun, "imu/acceleration", acceleration)
            log_vector(self.rerun, "imu/angular_velocity", angular_velocity)

    def register_scan(
        self,
        start_time: float,
        end_time: float,
        scan: np.ndarray,
        timestamps: np.ndarray,
    ):
        """
        Register a lidar scan.
        Timestamps are assumed to be absolute seconds.
        It is assumed there is sufficient IMU data added to the pipeline before triggering the registration (use the Sequencer).


        Parameters
        ----------
        start_time: float
            Absolute time of the scan recording start
        end_time: float
            Absolute time of the scan recording end
        scan : array of float, shape (N,3)
            Point cloud.
        timestamps : array of float, shape (N,)
            Absolute timestamps (seconds) for each point.
        """
        with ScopedProfiler("Pipeline - Registration") as registration_timer:
            if self.config.viz:
                # needs to be logged before the pybinded register function is called
                self.rerun.set_time("data_time", timestamp=end_time)
                stats = self.lio.interval_stats()
                self.rerun.log(
                    "imu/imu_count", self.rerun.Scalars(float(stats.imu_count))
                )
                log_vector(self.rerun, "imu/avg_acceleration", stats.avg_imu_accel())
                log_vector(
                    self.rerun, "imu/avg_body_acceleration", stats.avg_body_accel()
                )
                log_vector(self.rerun, "imu/avg_ang_velocity", stats.avg_ang_vel())

            try:
                if self.extrinsic_lidar2base is not None:
                    # TODO: rerun the deskewed scan as well, but there is some flickering in the viz for some reason
                    deskewed_scan = self.lio.register_scan_with_extrinsic(
                        self.extrinsic_lidar2base,
                        scan,
                        timestamps,
                    )
                else:
                    deskewed_scan = self.lio.register_scan(
                        scan,
                        timestamps,
                    )
            except ValueError as e:
                print(
                    "ERROR: Dropping LiDAR frame as there was an error. Odometry might suffer. Error:",
                    e,
                )
                return

            if self.config.dump_deskewed_scans:
                save_scan_as_ply(
                    deskewed_scan,
                    end_time,
                    output_dir=self.output_dir / "deskewed_scans",
                )

            if self.config.viz:
                with ScopedProfiler("Pipeline - Visualization") as _:
                    pose = self.lio.pose()
                    self.rerun.log(
                        "world/lidar",
                        self.rerun.Transform3D(
                            translation=pose[:3, 3],
                            mat3x3=pose[:3, :3],
                            axis_length=2,
                        ),
                    )
                    self.rerun.log(
                        "world/view_anchor",
                        self.rerun.Transform3D(translation=pose[:3, 3]),
                    )
                    traj_pts = np.array([self.last_xyz, pose[:3, 3]])
                    self.rerun.log(
                        "world/trajectory",
                        self.rerun.LineStrips3D(
                            [traj_pts], radii=[0.1], colors=[255, 111, 111]
                        ),
                    )
                    self.last_xyz = pose[:3, 3].copy()

                    self.viz_counter += 1
                    if self.viz_counter % self.config.viz_every_n_frames != 0:
                        # logging the point clouds is more expensive
                        # especially the local map, as we have to iterate over the entire map
                        # so we publish the lidar every n frames
                        return

                    local_map_points = self.lio.map_point_cloud()
                    if local_map_points.size > 0:
                        self.rerun.log(
                            "world/local_map",
                            self.rerun.Points3D(
                                local_map_points,
                                colors=height_colors_from_points(local_map_points),
                            ),
                        )

    def dump_results_to_disk(self):
        """
        Write LIO results to disk under LIOPipeline.output_dir.

        Writes:
        - Trajectory (timestamps and poses) in TUM format text file.
        - Configuration as YAML file.
        """
        traj_file = self.output_dir / f"{self.output_dir.name}_tum.txt"
        timestamps, poses = self.lio.poses_with_timestamps()
        with traj_file.open("w") as f:
            for t, p in zip(timestamps, poses):
                # p: x,y,z,qx,qy,qz,qw
                line = f"{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f} {p[4]:.6f} {p[5]:.6f} {p[6]:.6f}\n"
                f.write(line)
        info(f"Poses written to {traj_file.resolve()}")

        config = self.config.to_dict()
        config["log_dir"] = config["log_dir"].as_posix()
        settings_file = self.output_dir / "config.yaml"
        with settings_file.open("w") as f:
            yaml.dump(config, f, sort_keys=False)
        info(f"Configuration written to {settings_file.resolve()}")


def log_vector(rerun, entity_path_prefix: str, vector):
    """
    Logs a vector as three scalar time-series in rerun.

    Args:
        rerun: rerun module
        entity_path_prefix: Base path for scalar logs (e.g. "imu/avg_acceleration")
        vector: Iterable or np.ndarray with 3 elements (x, y, z)
    """
    rerun.log(f"{entity_path_prefix}/x", rerun.Scalars(vector[0]))
    rerun.log(f"{entity_path_prefix}/y", rerun.Scalars(vector[1]))
    rerun.log(f"{entity_path_prefix}/z", rerun.Scalars(vector[2]))


def log_vector_columns(
    rerun, entity_path_prefix: str, times: np.ndarray, vectors: np.ndarray
):
    """
    Log a batch of 3D vectors over multiple timestamps in rerun,
    sending one column batch per vector axis.

    Args:
        rerun: rerun module or rerun instance.
        entity_path_prefix: base path e.g. 'imu/acceleration'.
        times: 1D np.ndarray of timestamps (float64).
        vectors: 2D np.ndarray, shape (N, 3) where columns are x,y,z.
    """
    # Common time column to link all components
    time_col = rerun.TimeColumn("data_time", timestamp=times)

    # For each component, prepare scalar column and send
    for dim, axis_label in enumerate(["x", "y", "z"]):
        rerun.send_columns(
            f"{entity_path_prefix}/{axis_label}",
            indexes=[time_col],
            columns=rerun.Scalars.columns(scalars=vectors[:, dim]),
        )
