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

from pathlib import Path

from .rko_lio_pybind import (
    _LIOConfig,
    _TimestampProcessingConfig,
)
from .util import quat_xyzw_xyz_to_transform, transform_to_quat_xyzw_xyz


class TimestampProcessingConfig(_TimestampProcessingConfig):

    default_config = {
        "multiplier_to_seconds": 0.0,
        "force_absolute": False,
        "force_relative": False,
    }

    def __init__(self, **kwargs):
        """kwargs should match the keys expected in default_config"""
        super().__init__()
        cfg = dict(self.default_config)
        cfg.update(kwargs)
        for k, v in cfg.items():
            setattr(self, k, v)

    def to_dict(self):
        """Return a dict version"""
        return {k: getattr(self, k) for k in self.default_config.keys()}

    def __repr__(self):
        attrs = ", ".join(
            f"{k}={getattr(self, k)!r}" for k in self.default_config.keys()
        )
        return f"TimestampProcessingConfig({attrs})"


class LIOConfig(_LIOConfig):
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

    default_config = {
        "deskew": True,
        "max_iterations": 100,
        "voxel_size": 1.0,
        "max_points_per_voxel": 20,
        "max_range": 100.0,
        "min_range": 1.0,
        "convergence_criterion": 1e-5,
        "max_correspondance_distance": 0.5,
        "max_num_threads": 0,
        "initialization_phase": False,
        "max_expected_jerk": 3.0,
        "double_downsample": True,
        "min_beta": 200.0,
    }

    def __init__(self, **kwargs):
        """kwargs should match the keys expected in default_config"""
        super().__init__()
        cfg = dict(self.default_config)
        cfg.update(kwargs)
        for k, v in cfg.items():
            setattr(self, k, v)

    def to_dict(self):
        """Return a dict version"""
        return {k: getattr(self, k) for k in self.default_config.keys()}

    def __repr__(self):
        attrs = ", ".join(
            f"{k}={getattr(self, k)!r}" for k in self.default_config.keys()
        )
        return f"LIOConfig({attrs})"


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

    default_config = {
        "extrinsic_imu2base_quat_xyzw_xyz": None,
        "extrinsic_lidar2base_quat_xyzw_xyz": None,
        "viz": False,
        "viz_every_n_frames": 20,
        "dump_deskewed_scans": False,
        "log_dir": Path("results").resolve().as_posix(),
        "run_name": None,
    }

    def __init__(self, lio=None, timestamps=None, **kwargs):
        """kwargs should match the keys expected in default_config"""
        # ---- build LIO subconfig ----
        if isinstance(lio, dict):
            self.lio = LIOConfig(**lio)
        elif isinstance(lio, LIOConfig):
            self.lio = lio
        else:
            # collect any keys from kwargs intended for LIOConfig
            lio_args = {
                k: v for k, v in kwargs.items() if k in LIOConfig.default_config
            }
            self.lio = LIOConfig(**lio_args)

        # ---- build TimestampProcessing subconfig ----
        if isinstance(timestamps, dict):
            self.timestamps = TimestampProcessingConfig(**timestamps)
        elif isinstance(timestamps, TimestampProcessingConfig):
            self.timestamps = timestamps
        else:
            ts_args = {
                k: v
                for k, v in kwargs.items()
                if k in TimestampProcessingConfig.default_config
            }
            self.timestamps = TimestampProcessingConfig(**ts_args)

        # --- Pipeline parameters ---
        cfg = dict(self.default_config)
        cfg.update(kwargs)
        self.extrinsic_imu2base = None
        self.extrinsic_lidar2base = None
        if cfg.get("extrinsic_imu2base_quat_xyzw_xyz") is not None:
            self.extrinsic_imu2base = quat_xyzw_xyz_to_transform(
                cfg["extrinsic_imu2base_quat_xyzw_xyz"]
            )
        if cfg.get("extrinsic_lidar2base_quat_xyzw_xyz") is not None:
            self.extrinsic_lidar2base = quat_xyzw_xyz_to_transform(
                cfg["extrinsic_lidar2base_quat_xyzw_xyz"]
            )
        self.viz = cfg["viz"]
        self.viz_every_n_frames = cfg["viz_every_n_frames"]
        self.dump_deskewed_scans = cfg["dump_deskewed_scans"]
        self.log_dir = Path(cfg["log_dir"])
        self.run_name = cfg["run_name"]

    def to_dict(self):
        """Return full configuration as serializable dict."""
        d = dict(self.default_config)
        extrinsic_key_map = {
            "extrinsic_imu2base_quat_xyzw_xyz": "extrinsic_imu2base",
            "extrinsic_lidar2base_quat_xyzw_xyz": "extrinsic_lidar2base",
        }
        for key in self.default_config:
            if key in extrinsic_key_map:
                val = getattr(self, extrinsic_key_map[key])
                d[key] = transform_to_quat_xyzw_xyz(val) if val is not None else None
            else:
                if hasattr(self, key):
                    value = getattr(self, key)
                    if isinstance(value, Path):
                        value = value.resolve().as_posix()
                    d[key] = value
        d["lio"] = self.lio.to_dict()
        d["timestamps"] = self.timestamps.to_dict()
        return d

    @classmethod
    def default_dict(cls, include_timestamps: bool = False):
        """
        Parameters
        ----------
        include_timestamps : bool, default False
            If True, include the timestamp subconfig.
        """
        d = dict(cls.default_config)
        # remove the keys who's override behaviour will not be intuitive
        # or should not be specified in a user config
        d.pop("viz")
        d.pop("viz_every_n_frames")
        d["lio"] = dict(LIOConfig.default_config)
        if include_timestamps:
            d["timestamps"] = dict(TimestampProcessingConfig.default_config)
        return d

    def __repr__(self):
        return f"PipelineConfig({self.to_dict()!r})"
