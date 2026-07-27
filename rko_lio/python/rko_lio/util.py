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
    end_time_ns: int,
    output_dir: Path,
):
    """
    dumps the scan as PLY.
    The filename is <nanoseconds_as_int>.ply.
    """
    if scan is None or len(scan) == 0:
        return
    try:
        from plyfile import PlyData, PlyElement
    except ModuleNotFoundError:
        error_and_exit(
            'plyfile is required for dumping the deskewed scans. Please `pip install "rko_lio[all]"` or `pip install plyfile`.'
        )

    output_dir.mkdir(exist_ok=True, parents=True)
    fname = output_dir / f"{int(end_time_ns)}.ply"

    points = np.asarray(scan, dtype=np.float64)
    vertex = np.empty(len(points), dtype=[("x", "f8"), ("y", "f8"), ("z", "f8")])
    vertex["x"], vertex["y"], vertex["z"] = points[:, 0], points[:, 1], points[:, 2]
    PlyData([PlyElement.describe(vertex, "vertex")], text=False).write(fname.as_posix())
