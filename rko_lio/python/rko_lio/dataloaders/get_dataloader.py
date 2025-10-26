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


def dataloader_factory(name: str | None, data_path: Path, *args, **kwargs):
    if name is None:
        return guess_dataloader(data_path=data_path, *args, **kwargs)

    elif name == "rosbag":
        from .rosbag import RosbagDataLoader

        return RosbagDataLoader(data_path, *args, **kwargs)

    elif name == "raw":
        from .raw import RawDataLoader

        return RawDataLoader(data_path, *args, **kwargs)

    elif name == "helipr":
        from .helipr import HeliprDataLoader

        return HeliprDataLoader(data_path, *args, **kwargs)

    raise ValueError(f"Unknown dataloader: {name}")


def guess_dataloader(data_path: Path, *args, **kwargs):
    from ..util import error_and_exit, info

    # rosbag
    rosbag_exts = [".bag", ".db3", ".mcap"]
    for ext in rosbag_exts:
        matched_files = list(data_path.glob(f"*{ext}"))
        if matched_files:
            info("Guessed dataloader as rosbag!")
            return dataloader_factory("rosbag", data_path, *args, **kwargs)

    # raw data. Check if it contains
    #   - folder named 'lidar' and any .txt or .csv file
    #   - or a file named "rko_lio_settings.yaml" exists
    lidar_folder = data_path / "lidar"
    txt_files = list(data_path.glob("*.txt"))
    csv_files = list(data_path.glob("*.csv"))
    rko_lio_settings_file = data_path / "rko_lio_settings.yaml"
    if rko_lio_settings_file.exists() or (
        lidar_folder.is_dir() and (txt_files or csv_files)
    ):
        info("Guessed dataloader as raw!")
        return dataloader_factory("raw", data_path, *args, **kwargs)

    # helipr has a dataset specified file layout
    xsens_imu_path = data_path / "Inertial_data" / "xsens_imu.csv"
    lidar_folder = data_path / "LiDAR"

    if xsens_imu_path.is_file() and lidar_folder.is_dir():
        info("Guessed dataloader as Helipr!")
        return dataloader_factory("helipr", data_path, *args, **kwargs)

    # nothing guessed
    error_and_exit(
        f"Could not guess dataloader for path: {data_path}, please pass the loader with --dataloader or -d"
    )
