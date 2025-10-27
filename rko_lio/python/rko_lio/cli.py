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

from pathlib import Path

import typer

from .util import (
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

        from .config import PipelineConfig

        with open("config.yaml", "w") as f:
            yaml.dump(PipelineConfig.default_dict(), f, default_flow_style=False)
        info(
            "Default config dumped to config.yaml. Note that the extrinsics are left as an empty list. If you need them, you need to specify them as \[qx, qy, qz, qw, x, y, z]. Delete all the keys you don't need."
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
        help="Publish (rerun) LiDAR information after specified number of frames. A low value will slow down the entire pipeline as logging LiDAR data is expensive.",
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
        False,
        "--log",
        "-l",
        help="Log trajectory results to disk at 'log_dir' on completion",
        rich_help_panel="Disk logging options",
    ),
    log_dir: Path | None = typer.Option(
        None,
        "--log_dir",
        "-o",
        help="Where to dump LIO results if logging. If unspecified, and logging is enabled, a folder `results` will be created in the current directory.",
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
    dump_deskewed_scans: bool = typer.Option(
        False,
        "--dump_deskewed",
        help="Dump each deskewed/motion-undistorted scan as a .ply file under log_dir/run_name, only if logging with --log",
        rich_help_panel="Disk logging options",
    ),
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

    user_config = {}
    if config_fp:
        with open(config_fp, "r") as f:
            import yaml

            user_config.update(yaml.safe_load(f))
    user_config["log_dir"] = log_dir or user_config.get("log_dir", "results")
    user_config["run_name"] = run_name or user_config.get("run_name", data_path.name)
    user_config["dump_deskewed_scans"] = log_results and (
        dump_deskewed_scans or user_config.get("dump_deskewed_scans", False)
    )
    invalid_keys = ["viz", "viz_every_n_frames"]
    for key in invalid_keys:
        if key in user_config:
            warning(f"{key} specified in config will be ignored.")
    user_config["viz"] = viz
    user_config["viz_every_n_frames"] = viz_every_n_frames

    from .config import PipelineConfig

    pipeline_config = PipelineConfig(**user_config)

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
        timestamp_processing_config=pipeline_config.timestamps,
    )
    print("Loaded dataloader:", dataloader)

    user_ext_imu2base = pipeline_config.extrinsic_imu2base
    user_ext_lidar2base = pipeline_config.extrinsic_lidar2base
    if user_ext_imu2base is None or user_ext_lidar2base is None:
        info("Extrinsics missing or not fully specified in config.")
        dl_ext_imu2base, dl_ext_lidar2base = dataloader.extrinsics
        if user_ext_imu2base is None:
            pipeline_config.extrinsic_imu2base = dl_ext_imu2base
        if user_ext_lidar2base is None:
            pipeline_config.extrinsic_lidar2base = dl_ext_lidar2base

    if (
        pipeline_config.extrinsic_imu2base is None
        or pipeline_config.extrinsic_lidar2base is None
    ):
        error_and_exit(
            "Fatal: Could not obtain required IMU/Lidar extrinsics. Please specify in a config or as part of your data."
        )

    from .util import transform_to_quat_xyzw_xyz

    print("Resolved extrinsics:")
    print(
        "  IMU to Base:", transform_to_quat_xyzw_xyz(pipeline_config.extrinsic_imu2base)
    )
    print(
        "  Lidar to Base:",
        transform_to_quat_xyzw_xyz(pipeline_config.extrinsic_lidar2base),
    )

    from .lio_pipeline import LIOPipeline

    pipeline = LIOPipeline(pipeline_config)

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
