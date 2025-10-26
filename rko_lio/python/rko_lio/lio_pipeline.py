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

from .lio import LIO, LIOConfig
from .scoped_profiler import ScopedProfiler
from .util import (
    height_colors_from_points,
    info,
    save_scan_as_ply,
    transform_to_quat_xyzw_xyz,
)


class LIOPipeline:
    """
    Minimal sequential pipeline for LIO processing with out-of-sync IMU/lidar.
    Buffers are managed internally; data is added via add_imu and add_lidar.
    When IMU data covers an already available lidar frame, registration is triggered.
    """

    def __init__(
        self,
        config: LIOConfig,
        extrinsic_imu2base: np.ndarray | None = None,
        extrinsic_lidar2base: np.ndarray | None = None,
        viz: bool = False,
        viz_every_n_frames: int = 20,
        dump_deskewed_scans: bool = False,
        log_dir: Path = Path("results"), 
        run_name: str = "rko_lio_run",
    ):
        """
        Parameters
        ----------
        config : LIOConfig
            Config used to instantiate a LIO estimator.
        extrinsic_imu2base : np.ndarray[4,4] or None
            If provided, used for all IMU updates.
        extrinsic_lidar2base : np.ndarray[4,4] or None
            If provided, used for all scan registrations.
        """
        self.lio = LIO(config)
        self.extrinsic_imu2base = extrinsic_imu2base
        self.extrinsic_lidar2base = extrinsic_lidar2base

        # Each: dict with keys 'time', 'accel', 'gyro'
        self.imu_buffer: list[dict] = []
        # Each: dict with keys 'start_time', 'end_time', 'scan', 'timestamps'
        self.lidar_buffer: list[dict] = []

        self.dump_deskewed_scans = dump_deskewed_scans
        self.log_dir = log_dir
        self.run_name = run_name
        self._output_dir = None

        self.viz = viz
        if viz:
            import rerun

            self.rerun = rerun
            self.viz_counter = 0
            self.viz_every_n_frames = viz_every_n_frames
            self.last_xyz = np.zeros(3)

    @property
    def output_dir(self) -> Path:
        """
        The directory used for file logging if enabled.
        Folder is {log_dir}/{run_name}_{index}.
        Automatically bumps the index (from 0) if similar names exist, to avoid overwriting.
        """
        if self._output_dir is None:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            index = 0
            while True:
                output_dir = self.log_dir / f"{self.run_name}_{index}"
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
        self.imu_buffer.append(
            {
                "time": time,
                "acceleration": acceleration,
                "angular_velocity": angular_velocity,
            }
        )
        self._try_register()

    def add_lidar(
        self,
        scan: np.ndarray,
        timestamps: np.ndarray,
    ):
        """
        Add a lidar point cloud and *absolute* timestamps. Scan start and end times
        are inferred from the timestamps vector.

        Parameters
        ----------
        scan : array of float, shape (N,3)
            Point cloud.
        timestamps : array of float, shape (N,)
            Absolute timestamps (seconds) for each point.
        """

        start_time, end_time = np.min(np.asarray(timestamps)), np.max(
            np.asarray(timestamps)
        )
        self.lidar_buffer.append(
            {
                "scan": scan,
                "timestamps": timestamps,
                "start_time": start_time,
                "end_time": end_time,
            }
        )

        self._try_register()

    def _try_register(self):
        """
        Try to align and process available lidar with buffered imu.
        If latest IMU.timestamp > earliest lidar.end_time, start popping and then register.
        """
        while (
            self.lidar_buffer
            and self.imu_buffer
            and self.imu_buffer[-1]["time"] > self.lidar_buffer[0]["end_time"]
        ):
            with ScopedProfiler("Pipeline - Registration") as registration_timer:
                frame = self.lidar_buffer.pop(0)
                # Find all IMU measurements up to lidar end_time
                imu_to_process = [
                    imu for imu in self.imu_buffer if imu["time"] < frame["end_time"]
                ]
                for imu in imu_to_process:
                    if self.extrinsic_imu2base is not None:
                        self.lio.add_imu_measurement_with_extrinsic(
                            self.extrinsic_imu2base,
                            imu["acceleration"],
                            imu["angular_velocity"],
                            imu["time"],
                        )
                    else:
                        self.lio.add_imu_measurement(
                            imu["acceleration"],
                            imu["angular_velocity"],
                            imu["time"],
                        )
                # Remove processed IMUs from buffer (those with time < lidar end_time)
                self.imu_buffer = [
                    imu for imu in self.imu_buffer if imu["time"] >= frame["end_time"]
                ]
                # Register the lidar scan
                try:
                    if self.extrinsic_lidar2base is not None:
                        # TODO: rerun the deskewed scan as well, but there is some flickering in the viz for some reason
                        deskewed_scan = self.lio.register_scan_with_extrinsic(
                            self.extrinsic_lidar2base,
                            frame["scan"],
                            frame["timestamps"],
                        )
                    else:
                        deskewed_scan = self.lio.register_scan(
                            frame["scan"],
                            frame["timestamps"],
                        )
                except ValueError as e:
                    print(
                        "ERROR: Dropping LiDAR frame as there was an error. Odometry might suffer. Error:",
                        e
                    )
                    continue

            if self.dump_deskewed_scans:
                save_scan_as_ply(
                    deskewed_scan,
                    frame["end_time"],
                    output_dir=self.output_dir / "deskewed_scans",
                )

            if self.viz:
                with ScopedProfiler("Pipeline - Visualization") as viz_timer:
                    if self.lio.config.initialization_phase:
                        self.rerun.log(
                            "world",
                            self.rerun.ViewCoordinates.RIGHT_HAND_Z_UP,
                            static=True,
                        )
                    self.rerun.set_time("data_time", timestamp=frame["end_time"])
                    pose = self.lio.pose()
                    self.rerun.log(
                        "world/lidar",
                        self.rerun.Transform3D(
                            translation=pose[:3, 3], mat3x3=pose[:3, :3], axis_length=2
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
                    if self.viz_counter % self.viz_every_n_frames != 0:
                        # logging the point clouds is more expensive
                        # especially the local map, as we have to iterate over the entire map data structure
                        # so we publish the scans every n frames
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

        combined_config = {
            "lio": self.lio.config.to_dict(),
            "extrinsic_imu2base_quat_xyzw_xyz": (
                None
                if self.extrinsic_imu2base is None
                else transform_to_quat_xyzw_xyz(self.extrinsic_imu2base)
            ),
            "extrinsic_lidar2base_quat_xyzw_xyz": (
                None
                if self.extrinsic_lidar2base is None
                else transform_to_quat_xyzw_xyz(self.extrinsic_lidar2base)
            ),
            "viz": self.viz,
        }
        if self.viz:
            combined_config["viz_every_n_frames"] = self.viz_every_n_frames

        settings_file = self.output_dir / "settings.yaml"
        with settings_file.open("w") as f:
            f.write(
                "# please note that this file is not directly useable with rko_lio -c <config_file>.\n"
                "# please check the result from rko_lio --dump_config for the proper spec needed.\n\n"
            )
            yaml.dump(combined_config, f, sort_keys=False)
        info(f"Configuration written to {settings_file.resolve()}")
