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
Entrypoint typer application for the python wrapper.
"""

import sys
from pathlib import Path

import numpy as np
import typer

from .util import (
    error,
    error_and_exit,
    info,
    warning,
)


def version_callback(value: bool):
    if value:
        from importlib.metadata import version

        rko_lio_version = version("rko_lio")
        info("RKO_LIO Version:", rko_lio_version)
        raise typer.Exit(0)


def dump_config_callback(value: bool):
    if value:
        import yaml

        from .lio import LIOConfig

        def pybind_to_dict(obj):
            return {
                attr: getattr(obj, attr)
                for attr in dir(obj)
                if not attr.startswith("_") and not callable(getattr(obj, attr))
            }

        config = pybind_to_dict(LIOConfig())
        config["extrinsic_imu2base_quat_xyzw_xyz"] = []
        config["extrinsic_lidar2base_quat_xyzw_xyz"] = []

        with open("config.yaml", "w") as f:
            yaml.dump(config, f, default_flow_style=False)
        info(
            "Default config dumped to config.yaml. Note that the extrinsics are left as an empty list. If you don't need them, delete the two respective keys. If you need them, you need to specify them as \[qx, qy, qz, qw, x, y, z]."
        )
        raise typer.Exit(0)


def dataloader_name_callback(value: str):
    from .dataloaders import available_dataloaders

    if not value:
        return value
    dl = available_dataloaders()
    if value.lower() not in [d.lower() for d in dl]:
        raise typer.BadParameter(f"Supported dataloaders are: {', '.join(dl)}")
    for d in dl:
        if value.lower() == d.lower():
            return d
    return value


def parse_extrinsics_from_config(config_data: dict):
    """Parse extrinsics from config dict to 4x4 matrices."""
    extrinsic_imu2base = None
    extrinsic_lidar2base = None
    imu_config_key = "extrinsic_imu2base_quat_xyzw_xyz"
    lidar_config_key = "extrinsic_lidar2base_quat_xyzw_xyz"

    imu_config_val = config_data.pop(imu_config_key, None)
    lidar_config_val = config_data.pop(lidar_config_key, None)

    for key, valname in [
        (imu_config_val, imu_config_key),
        (lidar_config_val, lidar_config_key),
    ]:
        if key is not None and len(key) != 7:
            error_and_exit(f"Error: {valname} has length {len(key)} but should be 7.")

    from .util import quat_xyzw_xyz_to_transform

    if imu_config_val is not None:
        extrinsic_imu2base = quat_xyzw_xyz_to_transform(
            np.asarray(imu_config_val, dtype=np.float64)
        )
    if lidar_config_val is not None:
        extrinsic_lidar2base = quat_xyzw_xyz_to_transform(
            np.asarray(lidar_config_val, dtype=np.float64)
        )

    return extrinsic_imu2base, extrinsic_lidar2base


app = typer.Typer()


@app.command(
    epilog="Please open an issue on https://github.com/PRBonn/rko_lio if the usage of any option is unclear or you need some help!"
)
def cli(
    data_path: Path = typer.Argument(
        ...,
        exists=True,
        help="Path to data folder",
        file_okay=False,
        dir_okay=True,
        readable=True,
    ),
    config_fp: Path | None = typer.Option(
        None,
        "--config",
        "-c",
        exists=True,
        help="Path to config.yaml",
        file_okay=True,
        dir_okay=False,
        readable=True,
    ),
    dataloader_name: str | None = typer.Option(
        None,
        "--dataloader",
        "-d",
        help="Specify a dataloader: [rosbag, raw, helipr]. Leave empty to guess one",
        show_choices=True,
        callback=dataloader_name_callback,
        case_sensitive=False,
    ),
    viz: bool = typer.Option(
        False,
        "--viz",
        "-v",
        help="Enable Rerun visualization",
        rich_help_panel="Visualisation options",
    ),
    viz_every_n_frames: int = typer.Option(
        20,
        "--viz_frame_skip",
        help="Publish LiDAR information after (every) specified number of frames. A low value will slow down the entire pipeline as logging LiDAR is expensive.",
        rich_help_panel="Visualisation options",
    ),
    rbl_path: Path | None = typer.Option(
        None,
        "--rbl",
        exists=True,
        help="Path to a rerun blueprint file (.rbl). Leave empty to use the default rerun configuration. Respects --no_reset_viz if set",
        file_okay=True,
        dir_okay=False,
        readable=True,
        rich_help_panel="Visualisation options",
    ),
    reset_viz: bool = typer.Option(
        True,
        " /--no_reset_viz",
        help="Pass this option to disable resetting rerun viewer configuration as per the blueprint (default or with --rbl). Useful if you want to take advantage of rerun's caching behaviour.",
        show_default=False,
        rich_help_panel="Visualisation options",
    ),
    log_results: bool = typer.Option(
        True,
        "--log",
        "-l",
        help="Log trajectory results to disk at 'log_dir' on completion",
        rich_help_panel="Disk logging options",
    ),
    log_dir: Path | None = typer.Option(
        "results",
        "--log_dir",
        "-o",
        help="Where to dump LIO results if logging",
        file_okay=False,
        dir_okay=True,
        writable=True,
        rich_help_panel="Disk logging options",
    ),
    run_name: str | None = typer.Option(
        None,
        "--run_name",
        "-n",
        help="Name prefix for output files if logging. Leave empty to take the name from the data_path argument",
        rich_help_panel="Disk logging options",
    ),
    dump_deskewed_scans: bool = typer.Option(False, "--dump_deskewed",  help="Dump each deskewed/motion-undistorted scan as a .ply file under log_dir/run_name, only if logging with --log", rich_help_panel="Disk logging options"),
    sequence: str | None = typer.Option(
        None,
        "--sequence",
        help="Extra dataloader argument: sensor sequence",
        rich_help_panel="HeLiPR dataloader options",
    ),
    imu_topic: str | None = typer.Option(
        None,
        "--imu",
        help="Extra dataloader argument: imu topic",
        rich_help_panel="Rosbag dataloader options",
    ),
    lidar_topic: str | None = typer.Option(
        None,
        "--lidar",
        help="Extra dataloader argument: lidar topic",
        rich_help_panel="Rosbag dataloader options",
    ),
    base_frame: str | None = typer.Option(
        None,
        "--base_frame",
        help="Extra dataloader argument: base_frame for odometry estimation, default is lidar frame",
        rich_help_panel="Rosbag dataloader options",
    ),
    imu_frame: str | None = typer.Option(
        None,
        "--imu_frame",
        help="Extra dataloader argument: imu frame overload",
        rich_help_panel="Rosbag dataloader options",
    ),
    lidar_frame: str | None = typer.Option(
        None,
        "--lidar_frame",
        help="Extra dataloader argument: lidar frame overload",
        rich_help_panel="Rosbag dataloader options",
    ),
    version: bool | None = typer.Option(
        None,
        "--version",
        help="Print the current version of RKO_LIO and exit",
        callback=version_callback,
        is_eager=True,
        rich_help_panel="Auxilary commands",
    ),
    dump_config: bool | None = typer.Option(
        None,
        "--dump_config",
        help="Dump the default config to config.yaml and exit",
        callback=dump_config_callback,
        is_eager=True,
        rich_help_panel="Auxilary commands",
    ),
):
    """
    Run RKO_LIO with the selected dataloader and parameters.
    """

    if viz:
        try:
            import rerun as rr

            rr.init("rko_lio")
            rr.spawn(memory_limit="2GB")
            if reset_viz:
                rr.log_file_from_path(
                    Path(__file__).parent / "rko_lio.rbl"
                    if rbl_path is None
                    else rbl_path
                )

        except ImportError:
            error_and_exit(
                "Please install rerun with `pip install rerun-sdk` to enable visualization."
            )

    config_data = {}
    if config_fp:
        with open(config_fp, "r") as f:
            import yaml

            config_data.update(yaml.safe_load(f))

    from .dataloaders import dataloader_factory

    dataloader = dataloader_factory(
        name=dataloader_name,
        data_path=data_path,
        sequence=sequence,
        imu_topic=imu_topic,
        lidar_topic=lidar_topic,
        imu_frame_id=imu_frame,
        lidar_frame_id=lidar_frame,
        base_frame_id=base_frame,
    )
    print("Loaded dataloader:", dataloader)

    extrinsic_imu2base, extrinsic_lidar2base = parse_extrinsics_from_config(config_data)
    need_to_query_extrinsics = not all(
        T is not None and T.size != 0
        for T in (extrinsic_imu2base, extrinsic_lidar2base)
    )
    if need_to_query_extrinsics:
        warning(
            "One or both extrinsics are not specified in the config. Will try to obtain it from the data itself."
        )
        extrinsic_imu2base, extrinsic_lidar2base = dataloader.extrinsics
        print("Extrinsics obtained from dataloader.")
        from .util import transform_to_quat_xyzw_xyz

        def _print_extrinsic(name, T):
            print(f"{name}:\n\tTransform:")
            for row in T:
                print("\t\t" + np.array2string(row, precision=4, suppress_small=True))
            print("\tAs quat_xyzw_xyz:\n\t\t", transform_to_quat_xyzw_xyz(T), "\n")

        _print_extrinsic("IMU to Base", extrinsic_imu2base)
        _print_extrinsic("Lidar to Base", extrinsic_lidar2base)

    from .lio import LIOConfig

    config = LIOConfig(**config_data)

    from .lio_pipeline import LIOPipeline

    pipeline = LIOPipeline(
        config,
        extrinsic_imu2base=extrinsic_imu2base,
        extrinsic_lidar2base=extrinsic_lidar2base,
        viz=viz,
        log_dir=log_dir,
        run_name=run_name or data_path.name,
        dump_deskewed_scans=log_results and dump_deskewed_scans,
        viz_every_n_frames=viz_every_n_frames,
    )

    from tqdm import tqdm

    for kind, data_tuple in tqdm(dataloader, total=len(dataloader), desc="Data"):
        if kind == "imu":
            pipeline.add_imu(*data_tuple)
        elif kind == "lidar":
            pipeline.add_lidar(*data_tuple)

    if log_results:
        pipeline.dump_results_to_disk()


if __name__ == "__main__":
    app()
