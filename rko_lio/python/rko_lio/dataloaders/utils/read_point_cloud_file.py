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

from pathlib import Path

import numpy as np

# Recognised per-point time field names, shared with the ROS PointCloud2 reader.
from .ros_read_point_cloud import __TIMESTAMP_ATTRIBUTE_NAMES__ as TIME_FIELD_NAMES


def read_point_cloud_file(path):
    """
    Read a ``.ply`` point cloud and return ``(points, times)``.

    ``points`` is an ``(N, 3)`` float64 array. ``times`` is an ``(N,)`` float64
    array of per-point times if the cloud has a recognised time field (one of
    ``time``, ``timestamps``, ``timestamp``, ``t``), otherwise ``None``. Time
    units and absolute/relative handling are left to
    ``rko_lio_pybind._process_timestamps``, as in the other loaders.
    """
    path = Path(path)
    if path.suffix.lower() != ".ply":
        raise ValueError(f"Only .ply point clouds are supported, got '{path.suffix}': {path}")

    try:
        from plyfile import PlyData
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            'The raw dataloader needs plyfile. Install with `pip install "rko_lio[all]"` '
            "or `pip install plyfile`."
        ) from e

    vertex = PlyData.read(str(path))["vertex"].data
    names = vertex.dtype.names
    missing = [c for c in ("x", "y", "z") if c not in names]
    if missing:
        raise ValueError(
            f"Point cloud {path} is missing coordinate field(s) {missing}; has {names}"
        )
    points = np.column_stack([vertex["x"], vertex["y"], vertex["z"]]).astype(np.float64)
    for name in TIME_FIELD_NAMES:
        if name in names:
            return points, vertex[name].astype(np.float64)
    return points, None
