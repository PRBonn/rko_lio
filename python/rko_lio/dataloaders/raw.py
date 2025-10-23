"""
Raw Dataloader Usage
====================

This module provides the RawDataLoader class.

Example usage:

.. code-block:: python

    from rko_lio.dataloaders.raw import RawDataLoader
    loader = RawDataLoader("/path/to/data")
    print(loader.entries)
    # More on structure, YAML config, file layout, etc.

The loader expects ...
- instructions, scenarios, etc.
"""

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
from ..util import error_and_exit, info, warning

try:
    import open3d as o3d

except ImportError:
    error_and_exit(
        "Please install open3d with `pip install open3d` to use the raw dataloader."
    )


class RawDataLoader:
    def __init__(self, data_path: Path, *args, **kwargs):
        self.data_path = Path(data_path)

        self.rko_lio_settings = None
        if (self.data_path / "rko_lio_settings.yaml").exists():
            with open(self.data_path / "rko_lio_settings.yaml", "r") as f:
                self.rko_lio_settings = yaml.safe_load(f)

        # Load IMU data
        def default_imu_file():
            imu_files = list(self.data_path.glob("*.csv")) + list(
                self.data_path.glob("*.txt")
            )
            if len(imu_files) != 1:
                error_and_exit(
                    f"Expected exactly one IMU CSV/TXT in {self.data_path}, found: {imu_files}"
                )
            return imu_files[0].relative_to(self.data_path)

        imu_file = self.data_path / self._get_from_settings(
            "imu:path", default_imu_file
        )
        if not imu_file.exists():
            error_and_exit(f"Imu file {imu_file} does not exist.")
        self.imu_data = self._load_imu_data(imu_file)

        # load lidar filenames
        def default_lidar_dir():
            lidar_dir = self.data_path / "lidar"
            if not lidar_dir.is_dir():
                error_and_exit(f"Expected a folder called 'lidar' in {self.data_path}")
            return lidar_dir

        lidar_dir = self.data_path / self._get_from_settings(
            "lidar:path", default_lidar_dir
        )
        if (not lidar_dir.exists()) or (not lidar_dir.is_dir()):
            error_and_exit(
                f"Lidar dir {lidar_dir} does not exist or is not a directory."
            )
        self.lidar_data = self._load_lidar_data(lidar_dir)
        self.possible_timestamp_attribute_names = [
            "time",
            "timestamps",
            "timestamp",
            "t",
        ]
        user_specified_timestamp_attribute = self._get_from_settings(
            "lidar:timestamp_attribute", lambda: None
        )
        if user_specified_timestamp_attribute is not None:
            self.possible_timestamp_attribute_names.append(
                user_specified_timestamp_attribute
            )

        # Build a global, sorted list of all timestamps
        self.entries = []
        for imu in self.imu_data:
            self.entries.append(("imu", imu["timestamp"], imu))
        for lidar in self.lidar_data:
            self.entries.append(("lidar", lidar["timestamp"], lidar))
        self.entries.sort(key=lambda x: x[1])

        self.T_imu_to_base = None
        self.T_lidar_to_base = None

    def __len__(self):
        return len(self.entries)

    def _get_from_settings(self, key: str, default_callable, _warned_keys=set()):
        """
        warned_keys is a trick (hack) to have a global variable to prevent repeated warnings.
        will work as long as it remains the default value.
        """
        if not isinstance(self.rko_lio_settings, dict):
            return default_callable()
        query = key.split(":")
        ref = self.rko_lio_settings
        traversed = ""
        for sub_key in query:
            traversed += f"{sub_key}:"
            if sub_key not in ref:
                if key not in _warned_keys:
                    _warned_keys.add(key)
                    warning(
                        f"{traversed} missing in rko_lio_settings.yaml, using default value for {key}."
                    )
                return default_callable()
            ref = ref[sub_key]
        if isinstance(ref, dict):
            error_and_exit(
                f"Final value for {key} is still a dict in rko_lio_settings.yaml, ill-formed."
            )
        return ref

    @property
    def extrinsics(self):
        if self.T_imu_to_base is None or self.T_lidar_to_base is None:
            info("Trying to obtain extrinsics from the data.")
            # load extrinsics from file
            tf_file = self.data_path / "transforms.yaml"
            if not tf_file.is_file():
                error_and_exit(
                    f"Querying extrinsics automatically from the raw dataloader requires a transforms.yaml file in {self.data_path}"
                )

            with open(tf_file, "r") as f:
                tf_data = yaml.safe_load(f)

            required_keys = ["T_imu_to_base", "T_lidar_to_base"]
            for key in required_keys:
                if not isinstance(tf_data, dict) or key not in tf_data:
                    error_and_exit(
                        f"Querying extrinsics automatically from the raw dataloader requires a '{key}' matrix with that (key) name in transforms.yaml inside {self.data_path}"
                    )

            self.T_imu_to_base = np.array(tf_data["T_imu_to_base"], dtype=float)
            if not self.T_imu_to_base.shape == (4, 4):
                error_and_exit(
                    f"T_imu_to_base shape is {self.T_imu_to_base.shape}, expected 4x4"
                )

            self.T_lidar_to_base = np.array(tf_data["T_lidar_to_base"], dtype=float)
            if not self.T_lidar_to_base.shape == (4, 4):
                error_and_exit(
                    f"T_lidar_to_base shape is {self.T_lidar_to_base.shape}, expected 4x4"
                )

        return self.T_imu_to_base, self.T_lidar_to_base

    def _load_imu_data(self, imu_file: Path):
        # this perhaps looks a bit complicated. But really its because of the formatting and
        # all i'm doing is to have a way to easily map whatever key the user's csv files have
        # to an expected column name if someone can suggest a simpler alternative, I'll take it
        imu_data = []
        print("Loading IMU data.")
        with open(imu_file, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                imu_data.append(
                    {
                        "timestamp": int(
                            float(
                                row[
                                    self._get_from_settings(
                                        "imu:headers:timestamp", lambda: "timestamp"
                                    )
                                ]
                            )
                            * float(
                                self._get_from_settings(
                                    "imu:timestamp_multiplier_to_nanoseconds",
                                    1,  # default assumption is imu time is in nanoseconds
                                )
                            )
                        ),
                        "gyro": np.array(
                            [
                                float(
                                    row[
                                        self._get_from_settings(
                                            "imu:headers:gyro_x", lambda: "gyro_x"
                                        )
                                    ]
                                ),
                                float(
                                    row[
                                        self._get_from_settings(
                                            "imu:headers:gyro_y", lambda: "gyro_y"
                                        )
                                    ]
                                ),
                                float(
                                    row[
                                        self._get_from_settings(
                                            "imu:headers:gyro_z", lambda: "gyro_z"
                                        )
                                    ]
                                ),
                            ]
                        ),
                        "accel": np.array(
                            [
                                float(
                                    row[
                                        self._get_from_settings(
                                            "imu:headers:accel_x", lambda: "accel_x"
                                        )
                                    ]
                                ),
                                float(
                                    row[
                                        self._get_from_settings(
                                            "imu:headers:accel_y", lambda: "accel_y"
                                        )
                                    ]
                                ),
                                float(
                                    row[
                                        self._get_from_settings(
                                            "imu:headers:accel_z", lambda: "accel_z"
                                        )
                                    ]
                                ),
                            ]
                        ),
                    }
                )
        print("Loaded IMU data.")
        return imu_data

    def _load_lidar_data(self, lidar_dir: Path):
        file_type = self._get_from_settings(
            "lidar:file_suffix_with_dot", lambda: ".ply"
        )
        lidar_files = sorted(lidar_dir.glob("*" + file_type))
        if not len(lidar_files):
            error_and_exit(
                f"No files with file extension f{file_type} found in {lidar_dir}."
            )
        lidar_data = []
        for lf_name in lidar_files:
            timestamp = int(
                float(lf_name.stem)
                * float(
                    self._get_from_settings(
                        "lidar:file_timestamp_multiplier_to_nanoseconds", lambda: 1
                    )
                )
            )
            lidar_data.append({"timestamp": timestamp, "filename": lf_name})
        return lidar_data

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
                for attr_name in self.possible_timestamp_attribute_names:
                    if attr_name in ply.point:
                        timestamps = ply.point[attr_name].numpy().flatten()
                        break
                else:
                    # TODO: should not throw if deskew: false
                    error_and_exit(
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
