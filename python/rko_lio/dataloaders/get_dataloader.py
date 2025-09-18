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

import sys
from pathlib import Path


def available_dataloaders():
    return ["rosbag", "raw", "helipr"]


def get_dataloader(
    name: str | None,
    data_path: Path,
    sequence: str | None = None,
    imu_topic: str | None = None,
    lidar_topic: str | None = None,
    imu_frame_id: str | None = None,
    lidar_frame_id: str | None = None,
    base_frame_id: str | None = None,
    query_extrinsics: bool = True,
):
    if name is None:
        return guess_dataloader(
            data_path=data_path,
            sequence=sequence,
            imu_topic=imu_topic,
            lidar_topic=lidar_topic,
            imu_frame_id=imu_frame_id,
            lidar_frame_id=lidar_frame_id,
            base_frame_id=base_frame_id,
            query_extrinsics=query_extrinsics,
        )

    if name == "rosbag":
        from .rosbag import RosbagDataLoader

        return RosbagDataLoader(
            data_path,
            imu_topic=imu_topic,
            lidar_topic=lidar_topic,
            imu_frame_id=imu_frame_id,
            lidar_frame_id=lidar_frame_id,
            base_frame_id=base_frame_id,
            query_extrinsics=query_extrinsics,
        )

    elif name == "raw":
        from .raw import RawDataLoader

        return RawDataLoader(data_path)

    elif name == "helipr":
        from .helipr import HeliprDataLoader

        if sequence is None:
            print("[ERROR] HeliprDataLoader requires --sequence parameter")
            sys.exit(1)
        return HeliprDataLoader(data_path, sequence)

    else:
        raise ValueError(f"Unknown dataloader: {name}")


def guess_dataloader(
    data_path: Path,
    sequence: str | None = None,
    imu_topic: str | None = None,
    lidar_topic: str | None = None,
    imu_frame_id: str | None = None,
    lidar_frame_id: str | None = None,
    base_frame_id: str | None = None,
    query_extrinsics: bool = True,
):
    # Check for rosbag files
    rosbag_exts = [".bag", ".db3", ".mcap"]
    found_rosbag_file = False
    for ext in rosbag_exts:
        matched_files = list(data_path.glob(f"*{ext}"))
        if matched_files:
            found_rosbag_file = True
            break
    if found_rosbag_file:
        print("Guessed dataloader as rosbag!")
        return get_dataloader(
            "rosbag",
            data_path,
            imu_topic=imu_topic,
            lidar_topic=lidar_topic,
            imu_frame_id=imu_frame_id,
            lidar_frame_id=lidar_frame_id,
            base_frame_id=base_frame_id,
            query_extrinsics=query_extrinsics,
        )

    # Check for raw data
    # A folder named 'lidar' and a .txt or .csv file alongside (in data_path)
    lidar_folder = data_path / "lidar"
    txt_files = list(data_path.glob("*.txt"))
    csv_files = list(data_path.glob("*.csv"))
    if lidar_folder.is_dir() and (txt_files or csv_files):
        print("Guessed dataloader as raw!")
        return get_dataloader("raw", data_path, query_extrinsics=query_extrinsics)

    # Check for helipr data
    xsens_imu_path = data_path / "Inertial_data" / "xsens_imu.csv"
    lidar_folder = data_path / "LiDAR"

    if xsens_imu_path.is_file() and lidar_folder.is_dir():
        if sequence is None:
            print("[ERROR] HeLiPR dataLoader requires --sequence parameter")
            sys.exit(1)

        seq_folder = lidar_folder / sequence
        if not seq_folder.is_dir():
            print(f"[ERROR] Helipr sequence folder does not exist: {seq_folder}")
            sys.exit(1)

        print("Guessed dataloader as Helipr!")
        return get_dataloader(
            "helipr", data_path, sequence=sequence, query_extrinsics=query_extrinsics
        )

    # No matching dataloader found
    print(
        f"[ERROR] Could not guess dataloader for path: {data_path}, please pass the loader with --dataloader or -d"
    )
    sys.exit(1)
