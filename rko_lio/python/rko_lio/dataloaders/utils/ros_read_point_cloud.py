"""
This file is based on https://github.com/ros2/common_interfaces/blob/4bac182a0a582b5e6b784d9fa9f0dabc1aca4d35/sensor_msgs_py/sensor_msgs_py/point_cloud2.py
All rights reserved to the original authors: Tim Field and Florian Vahl.
"""

import sys
from typing import Iterable, List, Optional, Tuple

import numpy as np

__TIMESTAMP_ATTRIBUTE_NAMES__ = ["time", "timestamps", "timestamp", "t"]

_DATATYPES = {
    "int8": np.dtype(np.int8),
    "uint8": np.dtype(np.uint8),
    "int16": np.dtype(np.int16),
    "uint16": np.dtype(np.uint16),
    "int32": np.dtype(np.int32),
    "uint32": np.dtype(np.uint32),
    "float32": np.dtype(np.float32),
    "float64": np.dtype(np.float64),
}

DUMMY_FIELD_PREFIX = "unnamed_field"


def get_datatype_name(field) -> str:
    for attr_name, attr_value in vars(field).items():
        key_lower = attr_name.lower()
        if key_lower in _DATATYPES and attr_value == field.datatype:
            return key_lower
    raise ValueError(f"Unknown datatype code {field.datatype} for field {vars(field)}")


def dtype_from_fields(fields: Iterable, point_step: Optional[int] = None) -> np.dtype:
    """
    Convert a Iterable of sensor_msgs.msg.PointField messages to a np.dtype.
    :param fields: The point cloud fields.
                   (Type: iterable of sensor_msgs.msg.PointField)
    :param point_step: Point step size in bytes. Calculated from the given fields by default.
                       (Type: optional of integer)
    :returns: NumPy datatype
    """
    # Create a lists containing the names, offsets and datatypes of all fields
    field_names = []
    field_offsets = []
    field_datatypes = []
    for i, field in enumerate(fields):
        # Datatype as numpy datatype
        datatype = _DATATYPES[get_datatype_name(field)]
        # Name field
        name = (
            field.name
            if getattr(field, "name", "") != ""
            else f"{DUMMY_FIELD_PREFIX}_{i}"
        )
        # Handle fields with count > 1 by creating subfields with a suffix consiting
        # of "_" followed by the subfield counter [0 -> (count - 1)]
        assert field.count > 0, "Can't process fields with count = 0."
        for a in range(field.count):
            # Add suffix if we have multiple subfields
            if field.count > 1:
                subfield_name = f"{name}_{a}"
            else:
                subfield_name = name
            assert (
                subfield_name not in field_names
            ), "Duplicate field names are not allowed!"
            field_names.append(subfield_name)
            # Create new offset that includes subfields
            field_offsets.append(field.offset + a * datatype.itemsize)
            field_datatypes.append(datatype.str)

    # Create dtype
    dtype_dict = {
        "names": field_names,
        "formats": field_datatypes,
        "offsets": field_offsets,
    }
    if point_step is not None:
        dtype_dict["itemsize"] = point_step
    return np.dtype(dtype_dict)


def read_points(
    cloud,
    field_names: Optional[List[str]] = None,
    uvs: Optional[Iterable] = None,
    reshape_organized_cloud: bool = False,
) -> np.ndarray:
    """
    Read points from a sensor_msgs.PointCloud2 message.
    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_cloud: Returns the array as an 2D organized point cloud if set.
    :return: Structured NumPy array containing all points.
    """
    # Cast bytes to numpy array
    points = np.ndarray(
        shape=(cloud.width * cloud.height,),
        dtype=dtype_from_fields(cloud.fields, point_step=cloud.point_step),
        buffer=cloud.data,
    )

    # Keep only the requested fields
    if field_names is not None:
        assert all(
            field_name in points.dtype.names for field_name in field_names
        ), "Requests field is not in the fields of the PointCloud!"
        # Mask fields
        points = points[list(field_names)]

    # Swap array if byte order does not match
    if bool(sys.byteorder != "little") != bool(cloud.is_bigendian):
        points = points.byteswap(inplace=True)

    # Select points indexed by the uvs field
    if uvs is not None:
        # Don't convert to numpy array if it is already one
        if not isinstance(uvs, np.ndarray):
            uvs = np.fromiter(uvs, int)
        # Index requested points
        points = points[uvs]

    # Cast into 2d array if cloud is 'organized'
    if reshape_organized_cloud and cloud.height > 1:
        points = points.reshape(cloud.width, cloud.height)

    return points


def read_point_cloud(msg) -> Tuple[np.ndarray, np.ndarray | None]:
    """
    Extract poitns and timestamps from a PointCloud2 message.

    :return: Tuple of [points, timestamps]
        points: array of x, y z points, shape: (N, 3)
        timestamps: array of per-pixel timestamps, shape: (N,). returns None if no timestamps
    """
    field_names = ["x", "y", "z"]
    t_field = None
    for field in msg.fields:
        if field.name in __TIMESTAMP_ATTRIBUTE_NAMES__:
            t_field = field.name
            field_names.append(t_field)
            break

    points_structured = read_points(msg, field_names=field_names)
    points = np.column_stack(
        [points_structured["x"], points_structured["y"], points_structured["z"]]
    )

    # Remove nan if any
    # TODO: need to handle the change in timestamps too
    # points = points[~np.any(np.isnan(points), axis=1)]

    if t_field:
        timestamps = points_structured[t_field].astype(np.float64)
        return points.astype(np.float64), timestamps
    else:
        return points.astype(np.float64), None
