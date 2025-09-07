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

import numpy as np
import typer


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
    def convert_quat_xyzw_xyz_to_matrix(quat_xyzw_xyz: np.ndarray) -> np.ndarray:
        from scipy.spatial.transform import Rotation

        quat = quat_xyzw_xyz[:4]
        xyz = quat_xyzw_xyz[4:]
        rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rot.as_matrix()
        transform[:3, 3] = xyz
        return transform

    extrinsic_imu2base = None
    extrinsic_lidar2base = None

    imu_config_val = config_data.pop("extrinsic_imu2base_quat_xyzw_xyz", None)
    lidar_config_val = config_data.pop("extrinsic_lidar2base_quat_xyzw_xyz", None)

    if imu_config_val is not None:
        extrinsic_imu2base = convert_quat_xyzw_xyz_to_matrix(
            np.asarray(imu_config_val, dtype=np.float64)
        )
    if lidar_config_val is not None:
        extrinsic_lidar2base = convert_quat_xyzw_xyz_to_matrix(
            np.asarray(lidar_config_val, dtype=np.float64)
        )
    return extrinsic_imu2base, extrinsic_lidar2base


app = typer.Typer()


@app.command()
def cli(
    data_path: Path = typer.Argument(..., exists=True, help="Path to data folder"),
    config_fp: Path | None = typer.Option(
        None, "--config", "-c", exists=True, help="Path to config.yaml"
    ),
    dataloader_name: str | None = typer.Option(
        None,
        "--dataloader",
        "-d",
        help="Specify a dataloader: [rosbag, raw, helipr]. Leave empty to guess one.",
        show_choices=True,
        callback=dataloader_name_callback,
        case_sensitive=False,
    ),
    viz: bool = typer.Option(False, "--viz", "-v", help="Enable Rerun visualization"),
    log_results: bool = typer.Option(
        False,
        "--log",
        "-l",
        help="Log trajectory results to disk at 'results_dir' on completion",
    ),
    results_dir: Path | None = typer.Option(
        "results", "--results_dir", "-r", help="Where to dump LIO results if logging"
    ),
    run_name: str = typer.Option(
        "rko_lio_experiment",
        "--run_name",
        "-n",
        help="Name prefix for output files if logging",
    ),
    sequence: str | None = typer.Option(
        None,
        "--sequence",
        help="Extra dataloader argument: sensor sequence (for helipr only)",
    ),
    imu_topic: str | None = typer.Option(
        None, "--imu", help="Extra dataloader argument: imu topic (for rosbag only)"
    ),
    lidar_topic: str | None = typer.Option(
        None, "--lidar", help="Extra dataloader argument: lidar topic (for rosbag only)"
    ),
    base_frame: str | None = typer.Option(
        None,
        "--base_frame",
        help="Extra dataloader argument: base_frame for odometry estimation, default is lidar frame (for rosbag only)",
    ),
    imu_frame: str | None = typer.Option(
        None,
        "--imu_frame",
        help="Extra dataloader argument: imu frame overload (for rosbag only)",
    ),
    lidar_frame: str | None = typer.Option(
        None,
        "--lidar_frame",
        help="Extra dataloader argument: lidar frame overload (for rosbag only)",
    ),
):
    """
    Run LIO pipeline with the selected dataloader and parameters.
    """

    if viz:
        try:
            import rerun as rr

            rr.init("rko_lio")
            rr.spawn(memory_limit="2GB")
            rr.log_file_from_path(Path(__file__).parent / "rko_lio.rbl")

        except ImportError:
            raise ImportError(
                "Please install rerun with `pip install rerun-sdk` to enable visualization."
            )

    config_data = {}
    if config_fp:
        with open(config_fp, "r") as f:
            import yaml

            config_data.update(yaml.safe_load(f))

    extrinsic_imu2base, extrinsic_lidar2base = parse_extrinsics_from_config(config_data)
    need_to_query_extrinsics = not (
        extrinsic_imu2base is not None
        and len(extrinsic_imu2base)
        and extrinsic_lidar2base is not None
        and len(extrinsic_lidar2base)
    )
    if need_to_query_extrinsics:
        print(
            "Warning: One or both extrinsics are not specified in the config. Will try to obtain it from the data(loader) itself."
        )

    from .dataloaders import get_dataloader

    dataloader = get_dataloader(
        name=dataloader_name,
        data_path=data_path,
        sequence=sequence,
        imu_topic=imu_topic,
        lidar_topic=lidar_topic,
        imu_frame_id=imu_frame,
        lidar_frame_id=lidar_frame,
        base_frame_id=base_frame,
        query_extrinsics=need_to_query_extrinsics,
    )
    data_count = len(dataloader)
    print("Loaded dataloader:", dataloader)
    if need_to_query_extrinsics:
        extrinsic_imu2base, extrinsic_lidar2base = dataloader.extrinsics
        print("Extrinsics obtained from dataloader.")
        print("Imu to Base:")
        print(extrinsic_imu2base)
        print("Lidar to base:")
        print(extrinsic_lidar2base)

    from .lio import LIOConfig

    config = LIOConfig(**config_data)

    from .lio_pipeline import LIOPipeline

    pipeline = LIOPipeline(
        config,
        extrinsic_imu2base=extrinsic_imu2base,
        extrinsic_lidar2base=extrinsic_lidar2base,
        viz=viz,
    )

    from tqdm import tqdm

    from .scoped_profiler import ScopedProfiler

    for idx in tqdm(range(data_count), total=data_count, desc="Data"):
        with ScopedProfiler("Pipeline - Data Loader") as pipeline_timer:
            kind, data_tuple = dataloader[idx]

        if kind == "imu":
            pipeline.add_imu(*data_tuple)
        elif kind == "lidar":
            pipeline.add_lidar(*data_tuple)
        else:
            raise ValueError(f"data type {kind} is invalid from the dataloader")

    if log_results and results_dir:
        results_dir.mkdir(parents=True, exist_ok=True)
        pipeline.dump_results_to_disk(results_dir, run_name)
        print(f"Results dumped to: {results_dir} [prefix: {run_name}]")


if __name__ == "__main__":
    app()
