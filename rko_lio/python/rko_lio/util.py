import sys
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.panel import Panel

console = Console()


def error(*args):
    msg = " ".join(str(a) for a in args)
    console.print(
        Panel(msg, title="Error", border_style="red", expand=False, title_align="left")
    )


def error_and_exit(*args):
    msg = " ".join(str(a) for a in args)
    console.print(
        Panel(msg, title="Error", border_style="red", expand=False, title_align="left")
    )
    sys.exit(1)


def warning(*args):
    msg = " ".join(str(a) for a in args)
    console.print(
        Panel(
            msg,
            title="Warning",
            border_style="yellow",
            expand=False,
            title_align="left",
        )
    )


def info(*args):
    msg = " ".join(str(a) for a in args)
    console.print(
        Panel(msg, title="Info", border_style="cyan", expand=False, title_align="left")
    )


def height_colors_from_points(
    points: np.ndarray,
    color_map=np.array(
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
    ),
) -> np.ndarray:
    """
    Given Nx3 array of points, return Nx3 array of RGB colors (dtype uint8)
    mapped from the z values using the colormap.
    Default colormap is viridis.
    Colors are uint8 in range [0, 255].
    """
    z = points[:, 2]
    z_min, z_max = np.percentile(z, 1), np.percentile(z, 99)  # accounts for any noise
    z_clipped = np.clip(z, z_min, z_max)

    if z_max == z_min:
        norm_z = np.zeros_like(z)
    else:
        norm_z = (z_clipped - z_min) / (z_max - z_min)

    idx = norm_z * (len(color_map) - 1)
    idx_low = np.floor(idx).astype(int)
    idx_high = np.clip(idx_low + 1, 0, len(color_map) - 1)
    alpha = idx - idx_low
    colors = (
        (1 - alpha)[:, None] * color_map[idx_low] + alpha[:, None] * color_map[idx_high]
    ).astype(np.uint8)

    return colors


def transform_to_quat_xyzw_xyz(T: np.ndarray):
    """Convert 4x4 transform matrix to [qx, qy, qz, qw, x, y, z]."""

    from pyquaternion import Quaternion

    assert T.shape == (4, 4), "Transform must be 4x4"
    q = Quaternion(matrix=T[:3, :3])
    x, y, z = T[:3, 3]
    return [float(val) for val in (q.x, q.y, q.z, q.w, x, y, z)]


def quat_xyzw_xyz_to_transform(quat_xyzw_xyz: np.ndarray | list | None) -> np.ndarray:
    """Convert [qx, qy, qz, qw, x, y, z] to 4x4 transform."""
    if quat_xyzw_xyz is None:
        return np.eye(4, dtype=np.float64)

    from pyquaternion import Quaternion

    qx, qy, qz, qw = quat_xyzw_xyz[:4]
    xyz = quat_xyzw_xyz[4:]
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Quaternion(x=qx, y=qy, z=qz, w=qw).rotation_matrix
    transform[:3, 3] = xyz
    return transform


def save_scan_as_ply(
    scan: np.ndarray,
    end_time_seconds: float,
    output_dir: Path,
):
    """
    dumps the scan as PLY.
    The filename is <nanoseconds_as_int>.ply based on end_time_seconds.
    """
    if scan is None or len(scan) == 0:
        return
    try:
        import open3d
    except ModuleNotFoundError:
        error_and_exit(
            'Open3d is not installed, required for dumping the deskewed scans. Please "pip install -U open3d"'
        )

    output_dir.mkdir(exist_ok=True, parents=True)
    fname = output_dir / f"{int(end_time_seconds * 1e9)}.ply"

    pc = open3d.geometry.PointCloud()
    pc.points = open3d.utility.Vector3dVector(scan)
    open3d.io.write_point_cloud(fname.as_posix(), pc)


def log_vector(rerun, entity_path_prefix: str, vector):
    """
    Logs a vector as three scalar time-series in rerun.

    Args:
        rerun: rerun module
        entity_path_prefix: Base path for scalar logs (e.g. "imu/avg_acceleration")
        vector: Iterable or np.ndarray with 3 elements (x, y, z)
    """
    rerun.log(f"{entity_path_prefix}/x", rerun.Scalars(vector[0]))
    rerun.log(f"{entity_path_prefix}/y", rerun.Scalars(vector[1]))
    rerun.log(f"{entity_path_prefix}/z", rerun.Scalars(vector[2]))


def log_vector_columns(
    rerun, entity_path_prefix: str, times: np.ndarray, vectors: np.ndarray
):
    """
    Log a batch of 3D vectors over multiple timestamps in rerun,
    sending one column batch per vector axis.

    Args:
        rerun: rerun module or rerun instance.
        entity_path_prefix: base path e.g. 'imu/acceleration'.
        times: 1D np.ndarray of timestamps (float64).
        vectors: 2D np.ndarray, shape (N, 3) where columns are x,y,z.
    """
    # Common time column to link all components
    time_col = rerun.TimeColumn("data_time", timestamp=times)

    # For each component, prepare scalar column and send
    for dim, axis_label in enumerate(["x", "y", "z"]):
        rerun.send_columns(
            f"{entity_path_prefix}/{axis_label}",
            indexes=[time_col],
            columns=rerun.Scalars.columns(scalars=vectors[:, dim]),
        )
