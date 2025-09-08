<div align="center">
  <h1>RKO_LIO - LiDAR-Inertial Odometry<br />Without Sensor-Specific Modelling</h1>
</div>

<p align="center">
ROS Distros:
<br />
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_humble.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_humble.yaml/badge.svg?branch=master" alt="Humble" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_jazzy.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_jazzy.yaml/badge.svg?branch=master" alt="Jazzy" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_kilted.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_kilted.yaml/badge.svg?branch=master" alt="Kilted" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_rolling.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_rolling.yaml/badge.svg?branch=master" alt="Rolling" /></a>
</p>

<p align="center">
Python Bindings:
<br />
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2204.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2204.yaml/badge.svg?branch=master" alt="Ubuntu 22.04" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2204_arm.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2204_arm.yaml/badge.svg?branch=master" alt="Ubuntu 22.04 ARM" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2404.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2404.yaml/badge.svg?branch=master" alt="Ubuntu 24.04" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2404_arm.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2404_arm.yaml/badge.svg?branch=master" alt="Ubuntu 24.04 ARM" /></a>
<br />
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_macos_14.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_macos_14.yaml/badge.svg?branch=master" alt="macOS 14" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_macos_15.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_macos_15.yaml/badge.svg?branch=master" alt="macOS 15" /></a>
<br />
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_windows_2022.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_windows_2022.yaml/badge.svg?branch=master" alt="Windows 2022" /></a>
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/PRBonn/rko_lio/refs/heads/master/docs/example_multiple_platforms.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" />
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

## Quick Start

In case you already have a rosbag (ROS1 or ROS2) which contains a TF tree, you can inspect the results of our odometry system with the following two steps

```bash
pip install rko_lio rosbags rerun-sdk
```

`rko_lio` is our odometry package, `rosbags` is required for using our rosbag dataloader, and `rerun-sdk` is what we use for our optional visualizer.
After everything is installed, run

```bash
rko_lio -v /path/to/rosbag_folder # <- has to be a directory! with either *.bag files or metadata.yaml from ROS2
```

and you should be good to go! For some quick details, click below.

<details>
<summary>Click here for some more details on how to use RKO_LIO and how the above works!</summary>

For all possible CLI flags, please check `rko_lio --help`.

The `-v` flag enables visualization.

Our rosbag dataloader works with either ROS1 or ROS2 bags.
Note that we don't fully support running `rko_lio` in partial or incomplete bags.
ROS2 especially will need a `metadata.yaml` file.

By default, we assume there is just one IMU topic and one LiDAR topic in the bag, in which case we automatically pick up the topic names and proceed further.
If there are multiple topics per sensor, you will be prompted to select one via the `--imu` or `--lidar` flags which you can pass to `rko_lio`.

Next, we assume there is a (static) TF tree in the bag. If so, we take the frame ids from the message topics we just picked up, build a static TF tree, and then query it for the extrinsic from IMU to LiDAR.
By default, we assume the LiDAR frame to be the base frame for odometry. If you would like to use a different frame, you can pass the frame id with `--base_frame` (note the other options available with `--help`).
The TF tree will be queried for the appropriate transformations (if they exist in the bag!).

In case there is no TF tree in the bag, then you will have to manually specify the extrinsics for IMU to base and LiDAR to base, as these two are **required** parameters.
Leave one of the extrinsics as identity if you want the other one to be the frame of estimation (you will still have to specify both parameters).
You can specify the extrinsics via a config YAML file with the keys `extrinsic_imu2base_quat_xyzw_xyz` and `extrinsic_lidar2base_quat_xyzw_xyz`.
Pass this file to `rko_lio` using the `-c` flag.
Check `python/config/default.yaml` for all possible configuration options.

An example invocation would then be

```bash
# the config file has the sensor extrinsics
rko_lio -v -c config.yaml --imu imu_topic --lidar lidar_topic /path/to/rosbag_folder
```

For more install and usage instructions, please refer to the [python bindings readme](python#rko_lio---python-bindings).
</details>


## About

RKO_LIO is a LiDAR-inertial odometry system that is by design simple to deploy on different sensor configurations and robotic platforms with as minimal a change in configuration as necessary.

We have no restriction on which LiDAR you can use, and you can do so without changing any config (we've tested Velodyne, Ouster, Hesai, Livox, Robosense, Aeva sensors).
For using an IMU, we require only the accelerometer and gyroscope readings, the bare minimum.
You don't need to look up manufacturer spec sheets to provide noise specifications, etc.

All you need to provide is the extrinsic transformation between the IMU and LiDAR and you can start using our system for your LiDAR-inertial odometry needs!

## Setup

### ROS2

> We are working on getting the odometry package into the ROS index, so you can install it using system package managers instead of building from source.

We currently support ROS2 Humble, Jazzy and Kilted.

Clone the repository into your ROS workspace and then

```bash
# we use ninja to build by default
colcon build --packages-select rko_lio # --symlink-install --event-handlers console_direct+
```

To launch the odometry node:

```bash
ros2 launch rko_lio odometry.launch.py # config_file:=/path/to/a/config.yaml rviz:=true
```

Please refer to the [ROS readme](ros) for further ROS-specific details.

<details>
<summary>Build information</summary>


Note that we have some [default build configuration options](ros/colcon.pkg) which should automatically get picked up by colcon.
We have a few dependencies, but as long as these defaults apply, the package should build without any further consideration.
If you encounter any issues, please check [build.md](docs/build.md) for further details or open an issue afterwards.

</details>

## Python

The python interface to our system can be convenient to investigate recorded data offline as you don't need to setup a ROS environment first.

We provide wheels for Linux, macOS, and Windows.

You can install RKO_LIO by simply

```bash
pip install rko_lio
```

<details>
<summary>Optional dependencies</summary>

There's a few optional dependencies depending on what part of the interface you use.
E.g., inspecting rosbag data will require `rosbags`, and enabling visualization will require `rerun-sdk`; you will be prompted when a dependency is missing.
In case you don't mind pulling in a few additional dependencies and want everything available, instead run

```bash
pip install "rko_lio[all]"
```

</details>

Afterwards, check

```bash
rko_lio --help
```

You'll find further usage instructions [here](python#usage).

For instructions on how to build from source, please check [here](/python/README.md#build-from-source).

<details>
<summary><b>Please prefer the ROS version over the python version if you can</b></summary>

The ROS version is the intended way to use our odometry system on a robot.
The python version is slower than the ROS version, not on the odometry itself, but on how we read incoming data, i.e. dataloading.
Without getting into details, if you can, you should prefer using the ROS version.
We also provide a way to directly inspect and run our odometry on recorded rosbags (see offline mode in [ROS usage](ros#usage)) which has a performance benefit over the python version.
The python interface is merely meant to be a convenience.

</details>


## A note on transformations

It bears mentioning here our convention for specifying sensor extrinsics, the one parameter we do require you to provide.

Throughout this package, we refer to transformations using `transform_<from-frame>_to_<to-frame>` or `transform_<from-frame>2<to-frame>`.

By this, we mean a transformation that converts a vector expressed in the `<from-frame>` coordinate system to the `<to-frame>` coordinate system.

Mathematically, this translates to:

$$
\mathbf{v}^{\text{to}} = {}^{\text{to}} \mathbf{T}_{\text{from}}  \mathbf{v}^{\text{from}}
$$

The superscript on the vector indicates the frame in which the vector is expressed, and $${}^{\text{to}} \mathbf{T}_{\text{from}}$$ corresponds to `transform_<from-frame>_to_<to-frame>`.

## License

This project is free software made available under the MIT license. For details, see the [LICENSE](LICENSE) file.

## RA-L Submission

You can check out the branch `ral_submission` for the version of the code used for submission to RA-L.
Please note that that branch is meant to be an as-is reproduction of the code used during submission and is not supported.
The `master` and release versions are vastly improved, supported, and are the recommended way to use this system.

## Acknowledgements

<details>
<summary>KISS-ICP, Kinematic-ICP, Bonxai, PlotJuggler, Rerun</summary>

This package is inspired by and would not be possible without the work of [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp).
Additionally, we use and rely heavily on, either in the package itself or during development, [Bonxai](https://github.com/facontidavide/Bonxai), [PlotJuggler](https://github.com/facontidavide/PlotJuggler), [Rerun](https://github.com/rerun-io/rerun), and of course ROS itself.

A special mention goes out to [Rerun](https://rerun.io/) for providing an extremely easy-to-use but highly performative visualization system.
Without this, I probably would not have made a python interface at all.

</details>
