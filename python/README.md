# RKO_LIO - Python Bindings

The python interface/wrapper is a convenience tool to run the odometry offline on recorded data.

## Setup

### Install from PyPI

Simply

```bash
pip install rko_lio
```

We provide PyPI wheels for Linux, macOS, and Windows ([PyPI page](https://pypi.org/project/rko-lio)).

To be able to use `rko_lio` with any of the dataloaders or to enable visualization, you'll need to install additional dependencies.
You'll be prompted for specific packages as they become required during runtime.

For example, to use our rosbag dataloader and visualize the results, you'll need

```bash
pip install rko_lio rosbags rerun-sdk
```

If you want to install everything at once, do

```bash
pip install "rko_lio[all]"
```

### Build from Source

Clone the repository, `cd python`, and then

```bash
pip install .
```

To have all dependencies installed, run

```bash
pip install ".[all]"
```

Or use the convenience recipes provided in the [Makefile](Makefile)

```bash
make install # installs all optional deps
make editable # installs an editable version with all deps
```

For more advanced details about the build system and core dependencies, please see [build.md](../docs/build.md).

## Usage

There's a number of flags you can pass to `rko_lio`

```bash
rko_lio --help
```

will print all the usage information you'll need.

### Data

We have three dataloaders available currently: rosbag (both ROS1 and ROS2), raw, and HeLiPR.

By default, the dataloader is detected from the provided path, but you can explicitly specify it using the `-d` flag.

The system uses three key frames:
- IMU sensor frame
- LiDAR sensor frame
- Robot base frame

The extrinsic transformations
- `extrinsic_imu2base_quat_xyzw_xyz`
- `extrinsic_lidar2base_quat_xyzw_xyz`

are **required parameters**. You can provide them (with those keys) via a config file (`-c` option).
See the examples in the `config/` folder.

You can also override any other config parameters via the config file.
You can dump a default config using `rko_lio --dump_config`.

If you don't pass the extrinsic parameters, we try to infer them from the dataset directly, and assume the lidar frame as the base frame where applicable.
For more details, please expand the dataset specific sections below.

---

<details>
<summary>Raw dataloader</summary>

When working with raw data, the folder structure and file layout have to follow a specific convention, since we need both lidar and imu data.

Folder layout example:
```
dataset_root/
├── transforms.yaml                 # requires specific keys for extrinsics
├── imu.csv                         # or imu.txt, requires specific column names
└── lidar/                          # folder needs to be named the same
    ├── 1662622237000000000.ply     # filenames should be timestamps in nanoseconds
    ├── 1662622238000000000.ply
    └── ...
```

Requirements:
- transforms.yaml: must define two keys: Each should be a 4×4 transformation matrix specifying extrinsics to the base frame. (See [this](../README.md#a-note-on-transformations) for frame-order convention.)
  - `T_imu_to_base`
  - `T_lidar_to_base`

- imu file (`.csv` or `.txt`): exactly one file is expected. Columns must include: `timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z`. Additional columns may be present, but these are required. `timestamp` in particular is assumed to be in nanoseconds, rest are SI units.

- lidar folder (`lidar/`): contains scans as `.ply` files.
  - Filename: must be a timestamp (ns) corresponding to the end of recording for that scan, i.e., close to time of last recorded point. This timestamp is used along with the IMU timestamp to sort both sensor data into a common index, which is then processed sequentially by the odometry system.
  - Each `.ply` must include a time field for points. Accepted field names are: `time`, `timestamp`, `timestamps`, or `t`. This time must be in seconds, representing the absolute time of point collection.

PRs are welcome to improve this dataloader.

</details>

---

<details>
<summary>Rosbag dataloader</summary>

If you're working with rosbags, either ros1 or ros2 bags, there's a few reasonable conditions by which you can simply run

```bash
rko_lio /path/to/bag
```

and the system will work. These are:
- The bag contains only a single lidar and a single imu topic.
  - If multiple exist, you’ll be prompted to select one via the `--lidar` and/or `--imu` flags.
- The bag contains a TF tree with a static transform between the lidar and imu frames.
  - Note that we support only static TFs, on either the python bindings or the ROS version. Dynamic TF handling is out of the scope of the python bindings. I haven't really had a requirement where I need to handle a dynamic TF between the IMU and LiDAR, though I did consider how to. Open an issue if you need this supported on the ROS side.
- The frame names in the message header match the names in the TF tree, i.e., the lidar message header `frame_id` has to match a frame id in the TF tree. Similary for the imu.
  - Yes, there are cases where the frame ids don't match. And yes, because I ran into this problem myself, I provide a way to handle this case. Override the frame ids with the `--lidar_frame` or `--imu_frame` flags.

If the rosbag has no TF tree:
- First, please ask your data provider to include the TF tree.
- You can manually specify the extrinsics via the config (see [`config/leg_kilo.yaml`](/config/leg_kilo.yaml) or [`config/oxford_spires.yaml`](/config/oxford_spires.yaml) as references).
- Also: can dataset providers please include TF trees in bags by default? ~~makes no sense~~

</details>

---

<details>
<summary>HeLiPR dataloader</summary>

This is deprecated and planned to be removed in a future release. I'm prioritising documentation for other parts and other tasks. If you need it, open an issue.

</details>

---

### Configuration

All configurable parameters are defined in [`config/default.yaml`](/config/default.yaml).
For descriptions of each parameter, see [config.md](../docs/config.md).
