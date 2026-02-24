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
# TODO: extend the documentation to include pipeline config information, and perhaps the LIO config.
"""
The C++ backend (`rko_lio::core::process_timestamps`) tries to automatically
decide whether a LiDAR timestamp sequence is **absolute** (already aligned to
wall-clock time) or **relative** (offsets that must be shifted by the message
header time). If the data cannot be confidently classified as either, a
`std::runtime_error` is thrown:

.. code-block::

    Runtime Error: TimestampProcessingConfig does not cover this particular
    case of data. Please investigate, modify the config, or open an issue.


When this error occurs, you can potentially adjust the `timestamps` section of your
configuration file to fix the issue. Specifying one of `force_absolute` or `force_relative` should do the trick.

Example (default configuration)
-------------------------------

.. code-block:: yaml

    your other configuration keys here
    timestamps:
      multiplier_to_seconds: 0.0
      force_absolute: false
      force_relative: false

Description of parameters
--------------------------

- ``multiplier_to_seconds``:

    Factor applied to raw timestamp values to convert them to seconds.
    Secondsd and nanoseconds are detected automatically when this is 0.0 (default).
    Specify this for any other case.
    For example, if timestamps are in microseconds, use ``1e-6``.

- ``force_absolute``:

    If set, timestamps are always treated as absolute, bypassing heuristics.

- ``force_relative``:

    If set, timestamps are always interpreted as relative to the LiDAR message header time, bypassing heuristics.

.. note::

    First, absolute timestamps are checked for. Then, relative. If the heuristics are still not satisfied, then the above described error is thrown.
"""

from dataclasses import asdict, dataclass, fields
from pathlib import Path

from .rko_lio_pybind import (
    _LIOConfig,
    _TimestampProcessingConfig,
)
from .util import error_and_exit


@dataclass
class TimestampConfig:
    multiplier_to_seconds: float = 0.0
    force_absolute: bool = False
    force_relative: bool = False

    def to_pybind(self) -> _TimestampProcessingConfig:
        cfg = _TimestampProcessingConfig()
        for field in fields(self):
            setattr(cfg, field.name, getattr(self, field.name))
        return cfg

    def to_dict(self):
        return asdict(self)


@dataclass
class LIOConfig:
    """
    LIO configuration options.

    Parameters
    ----------
    deskew : bool, default True
        If True, perform scan deskewing.
    max_iterations : int, default 100
        Maximum optimization iterations for scan matching.
    voxel_size : float, default 1.0
        Size of map voxels (meters).
    max_points_per_voxel : int, default 20
        Maximum points stored per voxel.
    max_range : float, default 100.0
        Max usable range of lidar (meters).
    min_range : float, default 1.0
        Minimum usable range of lidar (meters).
    convergence_criterion : float, default 1e-5
        Convergence threshold for optimization.
    max_correspondance_distance : float, default 0.5
        Max distance for associating points (meters).
    max_num_threads : int, default 0
        Max thread count (0 = autodetect).
    initialization_phase : bool, default False
        Whether to initialize on the first two lidar message.
    max_expected_jerk : float, default 3.0
        Max expected IMU jerk (m/s^3).
    double_downsample : bool, default True
        Double downsamples the incoming scan before registering. Disabling for sparse LiDARs may improve results.
    min_beta : float, default 200.0
        Minimum scaling on the orientation regularisation weight. Set to -1 to disable the cost.
    """

    deskew: bool = True
    max_iterations: int = 100
    voxel_size: float = 1.0
    max_points_per_voxel: int = 20
    max_range: float = 100.0
    min_range: float = 1.0
    convergence_criterion: float = 1e-5
    max_correspondance_distance: float = 0.5
    max_num_threads: int = 0
    initialization_phase: bool = False
    max_expected_jerk: float = 3.0
    double_downsample: bool = True
    min_beta: float = 200.0

    def to_pybind(self) -> _LIOConfig:
        cfg = _LIOConfig()
        for field in fields(self):
            setattr(cfg, field.name, getattr(self, field.name))
        return cfg

    def to_dict(self):
        return asdict(self)


@dataclass
class PipelineConfig:
    """
    Configuration for the LIOPipeline, wrapping both LIO- and timestamp-related configs.

    Parameters
    ----------
    lio : dict or LIOConfig
    timestamps : dict or TimestampProcessingConfig
        Configuration for timestamp preprocessing.
    extrinsic_imu2base : np.ndarray[4,4] or None, optional
        Extrinsic transform from IMU frame to base frame.
    extrinsic_lidar2base : np.ndarray[4,4] or None, optional
        Extrinsic transform from lidar frame to base frame.
    viz : bool, default False
        Enable visualization using rerun.
    viz_every_n_frames : int, default 20
    dump_deskewed_scans : bool, default False
        Save deskewed scans to disk.
    log_dir : Path, default "results"
        Directory to store trajectory results.
    run_name : str, default "rko_lio_run"
        Name of the current run.
    """

    lio: LIOConfig | None = None
    timestamps: TimestampConfig | None = None
    extrinsic_imu2base_quat_xyzw_xyz: list | None = None
    extrinsic_lidar2base_quat_xyzw_xyz: list | None = None
    viz: bool = False
    viz_every_n_frames: int = 20
    dump_deskewed_scans: bool = False
    log_dir: Path = Path("results")
    run_name: str | None = None

    def __post_init__(self):
        if self.lio is None:
            self.lio = LIOConfig()
        if self.timestamps is None:
            self.timestamps = TimestampConfig()
        self.log_dir = Path(self.log_dir)

    @classmethod
    def from_dict(cls, args: dict):
        lio_args = args.pop("lio", {})
        ts_args = args.pop("timestamps", {})

        pipeline_args = {}
        for field in fields(cls):
            fname = field.name
            if fname in ("lio", "timestamps"):
                continue
            if fname in args:
                pipeline_args[fname] = args.pop(fname)

        lio_valid_names = [f.name for f in fields(LIOConfig)]
        for arg in args:
            if arg not in lio_valid_names:
                error_and_exit(
                    "Config argument", arg, "is not a valid key. Please remove."
                )
        lio_args.update(args)

        return cls(
            lio=LIOConfig(**lio_args),
            timestamps=TimestampConfig(**ts_args),
            **pipeline_args,
        )

    def to_dict(self):
        return asdict(self)
