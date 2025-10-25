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
import re
import warnings
from pathlib import Path

import numpy as np

from .. import rko_lio_pybind
from ..scoped_profiler import ScopedProfiler
from ..util import error_and_exit, info
from .helipr_file_reader_pybind import read_lidar_bin


class HeliprDataLoader:
    """
    Dataset loader for HeLiPR dataset.
    data_path: e.g. 'helipr/Bridge01'
    sequence: LiDAR sensor name: 'Aeva', 'Avia', 'Ouster', 'Velodyne'
    """

    def __init__(self, data_path: Path, sequence: str | None = None, *args, **kwargs):
        if sequence is None:
            error_and_exit("HeLiPR dataloader needs a --sequence argument.")

        warnings.warn(
            "HeliprDataLoader is deprecated and will be removed in a future (the next major) release.",
            DeprecationWarning,
            stacklevel=2,
        )

        self.read_lidar_bin = read_lidar_bin

        self.data_path = Path(data_path)
        self.sensor = sequence  # e.g. "Aeva"
        self._imu_data = self._load_imu()
        self._lidar_entries = self._find_lidar_bin_files()
        self.entries = self._build_entries()

        self.T_imu_to_base = None
        self.T_lidar_to_base = None

    @property
    def extrinsics(self):
        """Return (T_imu_to_base, T_lidar_to_base) for this sequence."""
        if self.T_imu_to_base is None or self.T_lidar_to_base is None:
            self._load_extrinsics()
        return self.T_imu_to_base, self.T_lidar_to_base

    def _load_extrinsics(self):
        """Load IMU->base and LIDAR->base transforms for HeLiPR."""

        info("Trying to obtain extrinsics from the data.")
        calib_dir = self.data_path / "Calibration"

        imu_ouster_path = calib_dir / "IMU_Ouster_extrinsic.txt"
        lidar_extrinsic_path = calib_dir / "LiDAR_extrinsic.txt"

        if not imu_ouster_path.is_file():
            error_and_exit(f"Missing {imu_ouster_path}")
        if not lidar_extrinsic_path.is_file():
            error_and_exit(f"Missing {lidar_extrinsic_path}")

        # Load transforms - assuming the dash in the file means "to"
        T_imu_to_ouster = parse_extrinsic_txt(imu_ouster_path)
        T_ouster_to_seq = parse_lidar_extrinsic(lidar_extrinsic_path, self.sensor)

        # Chain to get IMU to Lidar
        self.T_imu_to_base = T_ouster_to_seq @ T_imu_to_ouster
        self.T_lidar_to_base = np.eye(4)

    def _load_imu(self):
        imu_file = self.data_path / "Inertial_data" / "xsens_imu.csv"
        assert imu_file.exists(), "{imu_file} does not exist for data path {data_path}"
        imu_data = []
        with open(imu_file, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                ts = int(row[0])
                gyro = np.array(
                    [float(row[8]), float(row[9]), float(row[10])], dtype=np.float32
                )
                accel = np.array(
                    [float(row[11]), float(row[12]), float(row[13])], dtype=np.float32
                )
                imu_data.append({"timestamp": ts, "gyro": gyro, "accel": accel})
        return imu_data

    def _find_lidar_bin_files(self):
        lidar_dir = self.data_path / "LiDAR" / self.sensor
        bin_files = sorted(lidar_dir.glob("*.bin"))
        if not bin_files:
            error_and_exit(f"No .bin files found in {lidar_dir}")
        lidar_entries = []
        for binfile in bin_files:
            ts = int(binfile.stem)  # ns int timestamp
            lidar_entries.append({"timestamp": ts, "filename": binfile})
        return lidar_entries

    def _build_entries(self):
        entries = []
        for imu in self._imu_data:
            entries.append(("imu", imu["timestamp"], imu))
        for lidar in self._lidar_entries:
            entries.append(("lidar", lidar["timestamp"], lidar))
        entries.sort(key=lambda x: x[1])
        return entries

    def __len__(self):
        return len(self.entries)

    def __iter__(self):
        self._iter = iter(self.entries)
        return self

    def __next__(self):
        with ScopedProfiler("HeLiPR Dataloader") as data_timer:
            kind, _, data = next(self._iter)

            if kind == "imu":
                return (
                    "imu",
                    (
                        data["timestamp"] / 1e9,
                        data["accel"],
                        data["gyro"],
                    ),
                )
            elif kind == "lidar":
                header_stamp_sec = data["timestamp"] / 1e9
                points, raw_timestamps = self.read_lidar_bin(
                    str(data["filename"]), self.sensor
                )
                points_arr = np.asarray(points).reshape(-1, 3)
                _, _, abs_timestamps = rko_lio_pybind._process_timestamps(
                    rko_lio_pybind._VectorDouble(np.asarray(raw_timestamps)),
                    header_stamp_sec,
                )
                return ("lidar", (points_arr, np.asarray(abs_timestamps)))

    def __repr__(self):
        return f"HeliprDataLoader({self.data_path.name}, Sensor={self.sensor}, {len(self.entries)} entries)"


def parse_extrinsic_txt(path: Path) -> np.ndarray:
    """
    Parse an extrinsic calibration .txt file into a transform matrix.
    Assumes:
      - Inside the [ ] brackets, numbers are always space-separated.
      - Rotation block has exactly 9 floats, Translation has exactly 3 floats.
    """
    text = path.read_text()

    # Case-insensitive, allow newlines within brackets
    rot_match = re.search(
        r"rotation\s*:\s*\[([^\]]+)\]", text, re.IGNORECASE | re.DOTALL
    )
    trans_match = re.search(
        r"translation\s*:\s*\[([^\]]+)\]", text, re.IGNORECASE | re.DOTALL
    )

    if not rot_match:
        error_and_exit(f"No rotation block found in {path}")
    if not trans_match:
        error_and_exit(f"No translation block found in {path}")

    rot_vals = [float(x) for x in rot_match.group(1).split()]
    trans_vals = [float(x) for x in trans_match.group(1).split(",")]

    if len(rot_vals) != 9:
        error_and_exit(f"Expected 9 rotation values in {path}, got {len(rot_vals)}")
    if len(trans_vals) != 3:
        error_and_exit(
            f"Expected 3 translation values in {path}, got {len(trans_vals)}"
        )

    R_mat = np.array(rot_vals, dtype=float).reshape(3, 3)
    t_vec = np.array(trans_vals, dtype=float)

    T = np.eye(4, dtype=float)
    T[:3, :3] = R_mat
    T[:3, 3] = t_vec

    return T


def parse_lidar_extrinsic(path: Path, target_sensor: str) -> np.ndarray:
    """
    Parse LiDAR_extrinsic.txt for the block matching 'Ouster - <target_sensor>'
    and return it as a matrix.

    Assumes inside Rotation and Translation brackets, numbers are space-separated.
    Matches are case-insensitive and multiline.
    """
    text = path.read_text()

    # Regex pattern to find each block starting with [Ouster - <sensor> Extrinsic Calibration]
    # Capture rotation and translation blocks inside that section
    pattern = re.compile(
        rf"\[ouster\s*-\s*{re.escape(target_sensor)}\s*extrinsic\s*calibration\]\s*"
        r"(.*?)"
        r"(?=\[ouster|\Z)",  # lookahead for next block starting with [ouster or end of file
        re.IGNORECASE | re.DOTALL,
    )

    match = pattern.search(text)
    if not match:
        error_and_exit(f"No block for Ouster - {target_sensor} in {path}")

    block_text = match.group(1)

    # Extract rotation block
    rot_match = re.search(
        r"rotation\s*:\s*\[([^\]]+)\]", block_text, re.IGNORECASE | re.DOTALL
    )
    trans_match = re.search(
        r"translation\s*:\s*\[([^\]]+)\]", block_text, re.IGNORECASE | re.DOTALL
    )

    if not rot_match:
        error_and_exit(
            f"No rotation block found for Ouster - {target_sensor} in {path}"
        )
    if not trans_match:
        error_and_exit(
            f"No translation block found for Ouster - {target_sensor} in {path}"
        )

    rot_vals = [float(x) for x in rot_match.group(1).split()]
    trans_vals = [float(x) for x in trans_match.group(1).split()]

    if len(rot_vals) != 9:
        error_and_exit(
            f"Expected 9 rotation values for Ouster - {target_sensor} in {path}, got {len(rot_vals)}"
        )
    if len(trans_vals) != 3:
        error_and_exit(
            f"Expected 3 translation values for Ouster - {target_sensor} in {path}, got {len(trans_vals)}"
        )

    R_mat = np.array(rot_vals, dtype=float).reshape(3, 3)
    t_vec = np.array(trans_vals, dtype=float)

    T = np.eye(4, dtype=float)
    T[:3, :3] = R_mat
    T[:3, 3] = t_vec

    return T
