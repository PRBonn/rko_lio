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

import numpy as np
import pytest
import yaml

# plyfile is an optional dependency (rko_lio[all]); skip if unavailable.
pytest.importorskip("plyfile", reason="raw dataloader needs the optional 'plyfile'")

from rko_lio.config import TimestampConfig
from rko_lio.dataloaders.raw import RawDataLoader

IMU_HEADER = ["timestamp", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"]


def write_ply(path, xyz, time=None):
    from plyfile import PlyData, PlyElement

    dtype = [("x", "f4"), ("y", "f4"), ("z", "f4")]
    if time is not None:
        dtype.append(("t", "f8"))
    vertex = np.empty(len(xyz), dtype=dtype)
    vertex["x"], vertex["y"], vertex["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if time is not None:
        vertex["t"] = time
    PlyData([PlyElement.describe(vertex, "vertex")], text=False).write(str(path))


def build_dataset(root, scans, imu_rows=(), transforms=None):
    """scans: list of (header_ns, xyz (N,3), time (N,) or None)."""
    (root / "lidar").mkdir(parents=True)
    for header_ns, xyz, time in scans:
        write_ply(root / "lidar" / f"{header_ns}.ply", xyz, time)
    with (root / "imu.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(IMU_HEADER)
        writer.writerows(imu_rows)
    transforms = transforms or {
        "T_imu_to_base": np.eye(4).tolist(),
        "T_lidar_to_base": np.eye(4).tolist(),
    }
    with (root / "transforms.yaml").open("w") as f:
        yaml.safe_dump(transforms, f)
    return root


def load(root, timestamp_config=None):
    return RawDataLoader(root, timestamp_config=timestamp_config or TimestampConfig())


def lidar_frames(loader):
    return [payload for kind, payload in loader if kind == "lidar"]


def imu_frames(loader):
    return [payload for kind, payload in loader if kind == "imu"]


def test_no_time_field_falls_back_to_header(tmp_path):
    header_ns = 1_776_863_762_100_071_637  # 19-digit ns, must survive exactly
    xyz = np.arange(12, dtype=np.float32).reshape(4, 3)
    build_dataset(tmp_path, [(header_ns, xyz, None)])

    (frame,) = lidar_frames(load(tmp_path))
    assert np.array_equal(frame["scan"], xyz.astype(np.float64))
    assert frame["start_time_ns"] == header_ns
    assert frame["end_time_ns"] == header_ns
    assert np.all(frame["timestamps"] == header_ns)
    assert frame["timestamps"].dtype == np.int64


def test_relative_per_point_time_offsets_by_header(tmp_path):
    header_ns = 1_000_000_000_000
    xyz = np.zeros((10, 3), dtype=np.float32)
    rel_seconds = np.linspace(0.0, 0.09, 10)  # relative, well under a second
    build_dataset(tmp_path, [(header_ns, xyz, rel_seconds)])

    (frame,) = lidar_frames(load(tmp_path))
    expected = header_ns + np.round(rel_seconds * 1e9).astype(np.int64)
    assert np.array_equal(frame["timestamps"], expected)
    assert frame["start_time_ns"] == expected.min()
    assert frame["end_time_ns"] == expected.max()


def test_absolute_time_preserves_legacy_conversion(tmp_path):
    # Absolute epoch-second times reproduce the pre-refactor round(sec * 1e9).
    header_ns = 1_700_000_000_000_000_000
    xyz = np.zeros((5, 3), dtype=np.float32)
    abs_seconds = np.full(5, header_ns * 1e-9)
    build_dataset(tmp_path, [(header_ns, xyz, abs_seconds)])

    (frame,) = lidar_frames(load(tmp_path))
    expected = np.round(abs_seconds * 1e9).astype(np.int64)
    assert np.array_equal(frame["timestamps"], expected)


def test_imu_parsing_and_time_ordering(tmp_path):
    xyz = np.zeros((3, 3), dtype=np.float32)
    imu_rows = [
        [100, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0],
        [300, -0.1, -0.2, -0.3, -1.0, -2.0, -3.0],
    ]
    build_dataset(tmp_path, [(200, xyz, None)], imu_rows)

    # entries must be time-ordered: imu@100, lidar@200, imu@300
    assert [kind for kind, _ in load(tmp_path)] == ["imu", "lidar", "imu"]

    first = imu_frames(load(tmp_path))[0]
    assert first["time"] == 100
    assert np.allclose(first["angular_velocity"], [0.1, 0.2, 0.3])
    assert np.allclose(first["acceleration"], [1.0, 2.0, 3.0])


def test_imu_timestamp_preserved_exactly(tmp_path):
    # 19-digit ns IMU stamps must not be mangled through float.
    stamp = 1_776_863_762_100_071_637
    build_dataset(
        tmp_path,
        [(stamp + 1, np.zeros((2, 3), dtype=np.float32), None)],
        [[stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
    )
    assert imu_frames(load(tmp_path))[0]["time"] == stamp


def test_extrinsics_from_transforms(tmp_path):
    T_imu = np.eye(4)
    T_imu[:3, 3] = [1.0, 2.0, 3.0]
    T_lidar = np.eye(4)
    T_lidar[0, 0] = -1.0
    build_dataset(
        tmp_path,
        [(10, np.zeros((2, 3), dtype=np.float32), None)],
        transforms={"T_imu_to_base": T_imu.tolist(), "T_lidar_to_base": T_lidar.tolist()},
    )
    imu2base, lidar2base = load(tmp_path).extrinsics
    assert np.allclose(imu2base, T_imu)
    assert np.allclose(lidar2base, T_lidar)


def test_custom_imu_columns_via_settings(tmp_path):
    xyz = np.zeros((2, 3), dtype=np.float32)
    (tmp_path / "lidar").mkdir(parents=True)
    write_ply(tmp_path / "lidar" / "60.ply", xyz, None)
    with (tmp_path / "sensors.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ns", "wx", "wy", "wz", "ax", "ay", "az"])
        writer.writerow([50, 0.4, 0.5, 0.6, 7.0, 8.0, 9.0])
    with (tmp_path / "transforms.yaml").open("w") as f:
        yaml.safe_dump(
            {"T_imu_to_base": np.eye(4).tolist(), "T_lidar_to_base": np.eye(4).tolist()}, f
        )
    with (tmp_path / "rko_lio_settings.yaml").open("w") as f:
        yaml.safe_dump(
            {
                "imu": {
                    "path": "sensors.csv",
                    "headers": {
                        "timestamp": "t_ns",
                        "gyro_x": "wx", "gyro_y": "wy", "gyro_z": "wz",
                        "accel_x": "ax", "accel_y": "ay", "accel_z": "az",
                    },
                }
            },
            f,
        )

    (imu,) = imu_frames(load(tmp_path))
    assert imu["time"] == 50
    assert np.allclose(imu["angular_velocity"], [0.4, 0.5, 0.6])
    assert np.allclose(imu["acceleration"], [7.0, 8.0, 9.0])


def test_missing_imu_column_errors(tmp_path):
    xyz = np.zeros((2, 3), dtype=np.float32)
    (tmp_path / "lidar").mkdir(parents=True)
    write_ply(tmp_path / "lidar" / "1.ply", xyz, None)
    with (tmp_path / "imu.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "gyro_x"])  # missing most columns
        writer.writerow([1, 0.0])
    with (tmp_path / "transforms.yaml").open("w") as f:
        yaml.safe_dump(
            {"T_imu_to_base": np.eye(4).tolist(), "T_lidar_to_base": np.eye(4).tolist()}, f
        )
    with pytest.raises(SystemExit):
        load(tmp_path)


def test_unsupported_point_cloud_format_raises(tmp_path):
    from rko_lio.dataloaders.utils.read_point_cloud_file import read_point_cloud_file

    bogus = tmp_path / "cloud.pcd"
    bogus.write_bytes(b"not really a pcd")
    with pytest.raises(ValueError, match="Only .ply"):
        read_point_cloud_file(bogus)


def test_deskewed_scan_dump_roundtrips_exactly(tmp_path):
    # Deskewed-scan dump must stay float64 (no precision loss).
    from rko_lio.dataloaders.utils.read_point_cloud_file import read_point_cloud_file
    from rko_lio.util import save_scan_as_ply

    scan = (np.random.default_rng(0).standard_normal((300, 3)) * 40.0).astype(np.float64)
    save_scan_as_ply(scan, 42, tmp_path)
    back, times = read_point_cloud_file(tmp_path / "42.ply")
    assert back.dtype == np.float64
    assert np.array_equal(back, scan)  # exact, not approximate
    assert times is None


def test_settings_empty_blocks_use_defaults(tmp_path):
    # Empty (null) settings blocks must not crash: `imu:` / `lidar:` / `headers:`
    # parse to None in yaml.
    build_dataset(
        tmp_path,
        [(200, np.zeros((2, 3), dtype=np.float32), None)],
        [[100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
    )
    (tmp_path / "rko_lio_settings.yaml").write_text("imu:\nlidar:\n")
    (imu,) = imu_frames(load(tmp_path))
    assert imu["time"] == 100


def test_partial_header_override_keeps_defaults(tmp_path):
    # Overriding only some column names must leave the rest at their defaults.
    (tmp_path / "lidar").mkdir(parents=True)
    write_ply(tmp_path / "lidar" / "5.ply", np.zeros((2, 3), dtype=np.float32), None)
    with (tmp_path / "imu.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ts", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"])
        writer.writerow([7, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0])
    with (tmp_path / "transforms.yaml").open("w") as f:
        yaml.safe_dump(
            {"T_imu_to_base": np.eye(4).tolist(), "T_lidar_to_base": np.eye(4).tolist()}, f
        )
    (tmp_path / "rko_lio_settings.yaml").write_text("imu:\n  headers:\n    timestamp: ts\n")

    (imu,) = imu_frames(load(tmp_path))
    assert imu["time"] == 7  # remapped column
    assert np.allclose(imu["angular_velocity"], [0.1, 0.2, 0.3])  # default columns
