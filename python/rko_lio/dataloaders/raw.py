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

import csv
from pathlib import Path

import numpy as np
import yaml

from ..scoped_profiler import ScopedProfiler

try:
    import open3d as o3d

except ImportError:
    raise ImportError(
        "Please install open3d with `pip install open3d` to use the raw dataloader."
    )


__TIMESTAMP_ATTRIBUTE_NAMES__ = ["time", "timestamps", "timestamp", "t"]


class RawDataLoader:
    def __init__(self, data_path: Path, query_extrinsics: bool = True):
        self.data_path = Path(data_path)

        self.T_imu_to_base = None
        self.T_lidar_to_base = None
        if query_extrinsics:
            # load extrinsics from file
            tf_file = self.data_path / "transforms.yaml"
            if not tf_file.is_file():
                raise RuntimeError(f"Missing transforms.yaml in {self.data_path}")
            with open(tf_file, "r") as f:
                tf_data = yaml.safe_load(f)
            required_keys = ["T_imu_to_base", "T_lidar_to_base"]
            for key in required_keys:
                if key not in tf_data:
                    raise RuntimeError(
                        f"Missing '{key}' in transforms.yaml inside {self.data_path}"
                    )
            self.T_imu_to_base = np.array(tf_data["T_imu_to_base"], dtype=float)
            self.T_lidar_to_base = np.array(tf_data["T_lidar_to_base"], dtype=float)
            assert self.T_imu_to_base.shape == (4, 4), (
                "Improper T_imu_to_base shape "
                f"{self.T_imu_to_base.shape}, expected 4x4"
            )
            assert self.T_lidar_to_base.shape == (4, 4), (
                "Improper T_lidar_to_base shape"
                f"{self.T_lidar_to_base.shape}, expected 4x4"
            )

        # Load IMU file (must be exactly one CSV or TXT)
        imu_files = list(self.data_path.glob("*.csv")) + list(
            self.data_path.glob("*.txt")
        )
        if len(imu_files) != 1:
            raise RuntimeError(
                f"Expected exactly one IMU CSV/TXT in {self.data_path}, found: {imu_files}"
            )
        imu_file = imu_files[0]

        self.imu_data = []
        with open(imu_file, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.imu_data.append(
                    {
                        "timestamp": int(row["timestamp"]),
                        "gyro": np.array(
                            [
                                float(row["gyro_x"]),
                                float(row["gyro_y"]),
                                float(row["gyro_z"]),
                            ]
                        ),
                        "accel": np.array(
                            [
                                float(row["accel_x"]),
                                float(row["accel_y"]),
                                float(row["accel_z"]),
                            ]
                        ),
                    }
                )

        self.lidar_dir = self.data_path / "lidar"
        if not self.lidar_dir.is_dir():
            raise RuntimeError(f"Expected a folder called 'lidar' in {self.data_path}")
        lidar_files = sorted(self.lidar_dir.glob("*.ply"))
        self.lidar_data = []
        for lf in lidar_files:
            # Timestamp from filename
            ts = int(lf.stem)
            self.lidar_data.append({"timestamp": ts, "filename": lf})

        # Build a global, sorted list of all timestamps
        self.entries = []
        for imu in self.imu_data:
            self.entries.append(("imu", imu["timestamp"], imu))
        for lidar in self.lidar_data:
            self.entries.append(("lidar", lidar["timestamp"], lidar))
        self.entries.sort(key=lambda x: x[1])

    def __len__(self):
        return len(self.entries)

    @property
    def extrinsics(self):
        return self.T_imu_to_base, self.T_lidar_to_base

    def __iter__(self):
        self._iter = iter(self.entries)
        return self

    def __next__(self):
        with ScopedProfiler("Raw Dataloader") as data_timer:
            kind, _, data = next(self._iter)

            if kind == "imu":
                return "imu", (data["timestamp"] / 1e9, data["accel"], data["gyro"])
            elif kind == "lidar":
                ply = o3d.t.io.read_point_cloud(str(data["filename"]))
                # Find a field for per-point timestamp
                for attr_name in __TIMESTAMP_ATTRIBUTE_NAMES__:
                    if attr_name in ply.point:
                        timestamps = ply.point[attr_name].numpy().flatten()
                        break
                else:
                    # TODO: should not throw if deskew: false
                    raise RuntimeError(
                        f"No per-point timestamp attribute found in {data['filename']}. Please check the attributes."
                    )
                points = ply.point["positions"].numpy()
                return "lidar", (points, timestamps)

    def __repr__(self):
        imu_info = f"{len(self.imu_data)} IMU readings"
        lidar_info = f"{len(self.lidar_data)} lidar frames"
        path_info = f"path={self.data_path}"
        entry_info = f"{len(self.entries)} total entries"
        return f"RawDataLoader({path_info}, {imu_info}, {lidar_info}, {entry_info})"
