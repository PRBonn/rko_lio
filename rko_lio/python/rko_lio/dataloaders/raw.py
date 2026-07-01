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
Raw Dataloader
--------------

A file-based dataloader for datasets laid out as plain files:

.. code-block:: none

   dataset_root/
   ├── transforms.yaml        # required: 4x4 extrinsics
   ├── imu.csv                # required: one IMU file (.csv or .txt)
   ├── lidar/                 # required: one .ply per scan, named <ns>.ply
   │   ├── 1662622237000000000.ply
   │   └── ...
   └── rko_lio_settings.yaml  # optional: overrides (see below)

- ``transforms.yaml`` defines ``T_imu_to_base`` and ``T_lidar_to_base``, each a
  4x4 matrix. See :ref:`Extrinsics and conventions <data-extrinsics-convention>`.
- The IMU CSV has a header row; default columns are ``timestamp, gyro_x, gyro_y,
  gyro_z, accel_x, accel_y, accel_z`` (``timestamp`` in nanoseconds, rest SI).
  Extra columns are ignored.
- ``lidar/`` holds one ``.ply`` per scan; the filename stem is the scan
  timestamp in **nanoseconds**. A per-point time field (``time``, ``timestamps``,
  ``timestamp`` or ``t``) is used for deskewing when present, with units and
  absolute/relative detected automatically; clouds without one fall back to the
  filename stamp (disable deskewing with ``deskew: false`` in a ``-c`` config).

Defaults can be overridden with an optional ``rko_lio_settings.yaml``:

.. code-block:: yaml

   imu:
     path: imu.csv                            # IMU file, if not auto-detected
     timestamp_multiplier_to_nanoseconds: 1   # e.g. 1e9 if the CSV time is in seconds
     headers:                                 # map expected name -> your column name
       timestamp: my_stamp_col                # gyro_x/y/z, accel_x/y/z likewise
   lidar:
     path: lidar                              # lidar subfolder name
     file_suffix: .ply
     filename_timestamp_multiplier_to_nanoseconds: 1
"""

import csv
from pathlib import Path

import numpy as np
import yaml

from .. import rko_lio_pybind
from ..config import TimestampConfig
from ..scoped_profiler import ScopedProfiler
from ..util import error_and_exit, info, warning
# recognised per-point time field names, shared with the ROS PointCloud2 reader
from .utils.ros_read_point_cloud import __TIMESTAMP_ATTRIBUTE_NAMES__

try:
    from plyfile import PlyData
except ModuleNotFoundError:
    error_and_exit(
        'plyfile not installed for the raw dataloader, please install with "pip install -U plyfile"'
    )

__DEFAULT_IMU_COLUMNS__ = {
    "timestamp": "timestamp",
    "gyro_x": "gyro_x",
    "gyro_y": "gyro_y",
    "gyro_z": "gyro_z",
    "accel_x": "accel_x",
    "accel_y": "accel_y",
    "accel_z": "accel_z",
}


def _stamp_to_ns(value, multiplier: float) -> int:
    """
    Convert a timestamp string to integer nanoseconds.

    For the default (already-nanoseconds) case we parse the integer directly:
    a 19-digit ns stamp does not survive a round-trip through float64.
    """
    text = str(value)
    if multiplier == 1:
        return int(text) if text.lstrip("-").isdigit() else int(float(text))
    return int(float(text) * multiplier)


def read_point_cloud_file(path):
    """
    Read a ``.ply`` point cloud and return ``(points, times)``.

    ``points`` is an ``(N, 3)`` float64 array. ``times`` is an ``(N,)`` float64
    array of per-point times if the cloud has a recognised time field, else
    ``None`` (units/absolute-vs-relative are left to ``_process_timestamps``).
    """
    path = Path(path)
    if path.suffix.lower() != ".ply":
        error_and_exit(f"Only .ply point clouds are supported, got '{path.suffix}': {path}")

    vertex = PlyData.read(str(path))["vertex"].data
    names = vertex.dtype.names
    missing = [c for c in ("x", "y", "z") if c not in names]
    if missing:
        error_and_exit(
            f"Point cloud {path} is missing coordinate field(s) {missing}; has {names}"
        )
    points = np.column_stack([vertex["x"], vertex["y"], vertex["z"]]).astype(np.float64)
    for name in __TIMESTAMP_ATTRIBUTE_NAMES__:
        if name in names:
            return points, vertex[name].astype(np.float64)
    return points, None


class RawDataLoader:
    def __init__(
        self,
        data_path: Path,
        timestamp_config: TimestampConfig,
        *args,
        **kwargs,
    ):
        self.data_path = Path(data_path)
        self.timestamp_config = timestamp_config
        self.settings = self._load_settings()

        imu_data = self._load_imu(self._resolve_imu_file())
        lidar_data = self._load_lidar(self._resolve_lidar_dir())

        self.entries = sorted(
            [("imu", d["timestamp"], d) for d in imu_data]
            + [("lidar", d["timestamp"], d) for d in lidar_data],
            key=lambda e: e[1],
        )

        self.T_imu_to_base = None
        self.T_lidar_to_base = None

    def _load_settings(self) -> dict:
        settings_file = self.data_path / "rko_lio_settings.yaml"
        if settings_file.exists():
            with open(settings_file, "r") as f:
                return yaml.safe_load(f) or {}
        return {}

    def _resolve_imu_file(self) -> Path:
        override = (self.settings.get("imu") or {}).get("path")
        if override:
            imu_file = self.data_path / override
            if not imu_file.exists():
                error_and_exit(f"IMU file {imu_file} does not exist.")
            return imu_file

        candidates = sorted(self.data_path.glob("*.csv")) + sorted(
            self.data_path.glob("*.txt")
        )
        if len(candidates) != 1:
            error_and_exit(
                f"Expected exactly one IMU .csv/.txt file in {self.data_path}, "
                f"found: {[c.name for c in candidates]}. Set 'imu: {{path: ...}}' "
                "in rko_lio_settings.yaml to disambiguate."
            )
        return candidates[0]

    def _resolve_lidar_dir(self) -> Path:
        override = (self.settings.get("lidar") or {}).get("path", "lidar")
        lidar_dir = self.data_path / override
        if not lidar_dir.is_dir():
            error_and_exit(f"Lidar directory {lidar_dir} does not exist.")
        return lidar_dir

    def _load_imu(self, imu_file: Path) -> list:
        imu_cfg = self.settings.get("imu") or {}
        columns = {**__DEFAULT_IMU_COLUMNS__, **(imu_cfg.get("headers") or {})}
        ts_multiplier = float(imu_cfg.get("timestamp_multiplier_to_nanoseconds", 1))

        info(f"Loading IMU data from {imu_file}.")
        imu_data = []
        with open(imu_file, "r", newline="") as f:
            reader = csv.DictReader(f)
            missing = [c for c in columns.values() if c not in (reader.fieldnames or [])]
            if missing:
                error_and_exit(
                    f"IMU file {imu_file} is missing column(s) {missing}. "
                    f"Found columns: {reader.fieldnames}. Override names via "
                    "'imu: {headers: {...}}' in rko_lio_settings.yaml."
                )
            for row in reader:
                imu_data.append(
                    {
                        "timestamp": _stamp_to_ns(row[columns["timestamp"]], ts_multiplier),
                        "gyro": np.array(
                            [float(row[columns[f"gyro_{a}"]]) for a in "xyz"]
                        ),
                        "accel": np.array(
                            [float(row[columns[f"accel_{a}"]]) for a in "xyz"]
                        ),
                    }
                )
        return imu_data

    def _load_lidar(self, lidar_dir: Path) -> list:
        lidar_cfg = self.settings.get("lidar") or {}
        suffix = lidar_cfg.get("file_suffix", ".ply")
        ts_multiplier = float(
            lidar_cfg.get("filename_timestamp_multiplier_to_nanoseconds", 1)
        )
        files = sorted(lidar_dir.glob(f"*{suffix}"))
        if not files:
            error_and_exit(f"No '*{suffix}' files found in {lidar_dir}.")
        return [
            {"timestamp": _stamp_to_ns(f.stem, ts_multiplier), "filename": f}
            for f in files
        ]

    @property
    def extrinsics(self):
        if self.T_imu_to_base is None or self.T_lidar_to_base is None:
            info("Trying to obtain extrinsics from the data.")
            tf_file = self.data_path / "transforms.yaml"
            if not tf_file.is_file():
                error_and_exit(
                    f"The raw dataloader needs a transforms.yaml file in {self.data_path}."
                )
            with open(tf_file, "r") as f:
                tf_data = yaml.safe_load(f)

            self.T_imu_to_base = self._read_transform(tf_data, "T_imu_to_base", tf_file)
            self.T_lidar_to_base = self._read_transform(
                tf_data, "T_lidar_to_base", tf_file
            )
        return self.T_imu_to_base, self.T_lidar_to_base

    @staticmethod
    def _read_transform(tf_data, key: str, tf_file: Path) -> np.ndarray:
        if not isinstance(tf_data, dict) or key not in tf_data:
            error_and_exit(f"transforms.yaml in {tf_file} must contain a '{key}' matrix.")
        matrix = np.array(tf_data[key], dtype=float)
        if matrix.shape != (4, 4):
            error_and_exit(f"{key} shape is {matrix.shape}, expected 4x4.")
        return matrix

    def __len__(self):
        return len(self.entries)

    def __iter__(self):
        self._iter = iter(self.entries)
        return self

    def __next__(self):
        while True:
            with ScopedProfiler("Raw Dataloader"):
                kind, _, data = next(self._iter)
                if kind == "imu":
                    return "imu", {
                        "time": int(data["timestamp"]),
                        "acceleration": data["accel"],
                        "angular_velocity": data["gyro"],
                    }
                try:
                    return "lidar", self._read_lidar(data)
                except RuntimeError as e:
                    # _process_timestamps can throw; skip the frame like rosbag/helipr.
                    warning("Error processing lidar frame.", e)
                    continue

    def _read_lidar(self, data: dict) -> dict:
        header_stamp_ns = int(data["timestamp"])
        points, per_point_time = read_point_cloud_file(data["filename"])

        if per_point_time is not None and per_point_time.size > 0:
            start_ns, end_ns, abs_timestamps_ns = rko_lio_pybind._process_timestamps(
                rko_lio_pybind._VectorDouble(per_point_time),
                header_stamp_ns,
                self.timestamp_config.to_pybind(),
            )
            timestamps = np.asarray(abs_timestamps_ns, dtype=np.int64)
        else:
            if not hasattr(self, "_printed_timestamp_warning"):
                self._printed_timestamp_warning = True
                warning(
                    f"No per-point timestamps in {data['filename']} (and similar). "
                    "Odometry will suffer; disable deskewing with 'deskew: false' in a config."
                )
            start_ns = end_ns = header_stamp_ns
            timestamps = np.full(points.shape[0], header_stamp_ns, dtype=np.int64)

        return {
            "start_time_ns": start_ns,
            "end_time_ns": end_ns,
            "scan": points,
            "timestamps": timestamps,
        }

    def __repr__(self):
        n_imu = sum(1 for e in self.entries if e[0] == "imu")
        n_lidar = sum(1 for e in self.entries if e[0] == "lidar")
        return (
            f"RawDataLoader(path={self.data_path}, {n_imu} IMU readings, "
            f"{n_lidar} lidar frames)"
        )
