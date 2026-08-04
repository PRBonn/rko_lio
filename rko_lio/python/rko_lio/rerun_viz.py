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

import atexit
import threading
from pathlib import Path

import numpy as np

from .scoped_profiler import profile_func
from .util import error_and_exit

try:
    import rerun
    import rerun.blueprint as rrb
except ImportError:
    error_and_exit("Please install rerun with `pip install rerun-sdk` to enable visualization.")


VIRIDIS = np.array(
    [
        [68, 1, 84],
        [71, 44, 122],
        [59, 81, 139],
        [44, 113, 142],
        [33, 144, 140],
        [39, 173, 129],
        [92, 200, 99],
        [170, 220, 50],
        [253, 231, 37],
    ],
    dtype=np.uint8,
)


def height_colors(points: np.ndarray, color_map: np.ndarray = VIRIDIS) -> np.ndarray:
    z = points[:, 2]
    z_min, z_max = np.percentile(z, 1), np.percentile(z, 99)
    z_clipped = np.clip(z, z_min, z_max)

    if z_max == z_min:
        norm_z = np.zeros_like(z)
    else:
        norm_z = (z_clipped - z_min) / (z_max - z_min)

    idx = norm_z * (len(color_map) - 1)
    idx_low = np.floor(idx).astype(int)
    idx_high = np.clip(idx_low + 1, 0, len(color_map) - 1)
    alpha = idx - idx_low
    return ((1 - alpha)[:, None] * color_map[idx_low] + alpha[:, None] * color_map[idx_high]).astype(np.uint8)


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


def default_blueprint():
    def since_start():
        return rrb.VisibleTimeRanges(
            [
                rrb.VisibleTimeRange(
                    "data_time",
                    start=rrb.TimeRangeBoundary.infinite(),
                    end=rrb.TimeRangeBoundary.cursor_relative(),
                )
            ]
        )

    def last_seconds(seconds: float):
        return rrb.VisibleTimeRanges(
            [
                rrb.VisibleTimeRange(
                    "data_time",
                    start=rrb.TimeRangeBoundary.cursor_relative(seconds=-seconds),
                    end=rrb.TimeRangeBoundary.cursor_relative(),
                )
            ]
        )

    local_map = rrb.Spatial3DView(
        name="Local Map",
        origin="/world/view_anchor",
        contents=["+ /world/**", "+ $origin/**", "- /__properties/**"],
        background=[0x45, 0x45, 0x45],
        line_grid=False,
        overrides={
            "/world/trajectory": [
                since_start(),
                rerun.LineStrips3D.from_fields(radii=0.1, colors=0xFF6F6FFF),
            ],
            "/world/deskewed_scan": rerun.Points3D.from_fields(colors=0xD25757FF),
            "/world/local_map": rerun.Points3D.from_fields(radii=-1.25),
        },
    )

    timeseries = rrb.Tabs(
        rrb.Vertical(
            rrb.TimeSeriesView(
                name="Acceleration",
                contents=[
                    "+ /imu/acceleration/**",
                    "+ /imu/avg_acceleration/**",
                    "- /__properties/**",
                ],
                time_ranges=last_seconds(2),
            ),
            rrb.TimeSeriesView(
                name="Angular Velocity",
                contents=[
                    "+ /imu/angular_velocity/**",
                    "+ /imu/avg_ang_velocity/**",
                    "- /__properties/**",
                ],
                time_ranges=last_seconds(2),
            ),
            name="IMU",
        ),
        rrb.TimeSeriesView(
            name="Body Acceleration",
            contents=["+ /imu/avg_body_acceleration/**", "- /__properties/**"],
            time_ranges=last_seconds(2),
        ),
        rrb.TimeSeriesView(
            name="Debug",
            contents=["+ /imu/imu_count", "- /__properties/**"],
        ),
        name="Timeseries",
    )

    return rrb.Blueprint(
        rrb.Tabs(local_map, timeseries),
        rrb.SelectionPanel(state="hidden"),
    )


class Viz:
    """
    Spawns the rerun viewer and logs everything the pipeline hands it.
    Point clouds go out on a worker thread via send_columns, which binds the
    timestamp to the data, so the worker never races the main thread's set_time.
    """

    def __init__(
        self,
        map_log_period_s: float = 1.0,
        rbl_path: Path | None = None,
        reset: bool = True,
        gravity_aligned: bool = True,
    ):
        rerun.init("rko_lio")
        rerun.spawn(memory_limit="2GB")
        if reset:
            if rbl_path is None:
                rerun.send_blueprint(default_blueprint())
            else:
                rerun.log_file_from_path(rbl_path)
        if gravity_aligned:
            rerun.log("world", rerun.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        self.map_log_period_s = map_log_period_s
        self.last_xyz = np.zeros(3)
        self.last_map_log_s = -float("inf")
        self.cloud_box = LatestMailbox()
        self.cloud_thread = threading.Thread(target=self.cloud_log_loop, daemon=True)
        self.cloud_thread.start()
        atexit.register(self.close)

    def log_vector(self, entity_path_prefix: str, vector):
        rerun.log(f"{entity_path_prefix}/x", rerun.Scalars(vector[0]))
        rerun.log(f"{entity_path_prefix}/y", rerun.Scalars(vector[1]))
        rerun.log(f"{entity_path_prefix}/z", rerun.Scalars(vector[2]))

    def log_imu(self, time_ns: int, acceleration, angular_velocity):
        rerun.set_time("data_time", timestamp=time_ns * 1e-9)
        self.log_vector("imu/acceleration", acceleration)
        self.log_vector("imu/angular_velocity", angular_velocity)

    def log_interval_stats(self, time_ns: int, stats):
        rerun.set_time("data_time", timestamp=time_ns * 1e-9)
        rerun.log("imu/imu_count", rerun.Scalars(float(stats.imu_count)))
        self.log_vector("imu/avg_acceleration", stats.avg_imu_accel())
        self.log_vector("imu/avg_body_acceleration", stats.avg_body_accel())
        self.log_vector("imu/avg_ang_velocity", stats.avg_ang_vel())

    def wants_local_map(self, time_ns: int) -> bool:
        """
        querying the map is expensive.
        get it and log every map_log_period_s
        """
        return time_ns * 1e-9 - self.last_map_log_s >= self.map_log_period_s

    @profile_func("Pipeline - Visualization")
    def log_frame(
        self,
        time_ns: int,
        pose: np.ndarray,
        deskewed_scan: np.ndarray,
        extrinsic_lidar2base: np.ndarray,
        local_map: np.ndarray | None,
    ):
        scan_time_s = time_ns * 1e-9
        rerun.set_time("data_time", timestamp=scan_time_s)
        rerun.log(
            "world/base",
            rerun.Transform3D(translation=pose[:3, 3], mat3x3=pose[:3, :3]),
        )
        rerun.log("world/base", rerun.TransformAxes3D(2.0))
        rerun.log("world/view_anchor", rerun.Transform3D(translation=pose[:3, 3]))
        rerun.log(
            "world/trajectory",
            rerun.LineStrips3D([np.array([self.last_xyz, pose[:3, 3]])]),
        )
        self.last_xyz = pose[:3, 3].copy()

        if local_map is not None and local_map.size > 0:
            self.last_map_log_s = scan_time_s
        else:
            local_map = None

        self.cloud_box.put((scan_time_s, deskewed_scan, pose @ extrinsic_lidar2base, local_map))

    def cloud_log_loop(self):
        for item in iter(self.cloud_box.get, None):
            scan_time_s, deskewed_scan, T_lidar2world, local_map = item
            scan_world = (T_lidar2world[:3, :3] @ deskewed_scan.T).T + T_lidar2world[:3, 3]
            time_idx = [rerun.TimeColumn("data_time", timestamp=[scan_time_s])]
            rerun.send_columns(
                "world/deskewed_scan",
                indexes=time_idx,
                columns=rerun.Points3D.columns(positions=scan_world).partition([len(scan_world)]),
            )
            if local_map is not None:
                rerun.send_columns(
                    "world/local_map",
                    indexes=time_idx,
                    columns=rerun.Points3D.columns(
                        positions=local_map,
                        colors=height_colors(local_map),
                    ).partition([len(local_map)]),
                )

    def close(self):
        self.cloud_box.close()
        self.cloud_thread.join()
