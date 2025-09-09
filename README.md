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
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_windows_11_arm.yaml"><img src="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_windows_11_arm.yaml/badge.svg?branch=master" alt="Windows 11 ARM" /></a>
</p>

<p align="center">
  <a href="https://www.youtube.com/watch?v=QrVHip5opIc">
    <img src="https://raw.githubusercontent.com/PRBonn/rko_lio/refs/heads/master/docs/example_multiple_platforms.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" />
  </a>
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

## Quick Start

### Python

In case you already have a rosbag (ROS1 or ROS2) which contains a TF tree, you can inspect the results of our odometry system with the following two steps

```bash
pip install rko_lio rosbags rerun-sdk
```

`rko_lio` is our odometry package, `rosbags` is required for using our rosbag dataloader, and `rerun-sdk` is what we use for our optional visualizer.
Next, run

```bash
rko_lio -v /path/to/rosbag_folder # <- has to be a directory! with either *.bag files or metadata.yaml from ROS2
```

and you should be good to go!

<details>
<summary><b>Click here for some more details on how the above works and how to use RKO_LIO!</b></summary>
<br />

The `-v` flag enables visualization.

You can specify a dataloader to use with `-d`, but if you don't, we try to guess the format based on the layout of the data.

Our rosbag dataloader works with either ROS1 or ROS2 bags.
Place split ROS1 bags in a single folder and pass the folder as the data path.
Note that we don't support running `rko_lio` on partial or incomplete bags, though you can try (and maybe raise an issue if you think we should support this).
ROS2 especially will need a `metadata.yaml` file.

By default, we assume there is just one IMU topic and one LiDAR topic in the bag, in which case we automatically pick up the topic names and proceed further.
If there are multiple topics per sensor, you will be prompted to select one via the `--imu` or `--lidar` flags, which you can pass to `rko_lio`.

Next, we assume there is a (static) TF tree in the bag.
If so, we take the frame ids from the message topics we just picked up, build a static TF tree, and then query it for the extrinsic from IMU to LiDAR.
Our odometry estimates the robot pose with respect to a base frame, and by default, we assume the LiDAR frame to be the base frame.
If you would like to use a different frame, you can pass the frame id with `--base_frame` (note the other options available with `--help`).
The TF tree will be queried for the appropriate transformations (if they exist in the bag!).

In case there is no TF tree in the bag, then you will have to manually specify the extrinsics for IMU to base frame and LiDAR to base frame, as these two are **required** parameters.
Set one of the extrinsics to identity if you want that one to be the base frame (you will still have to specify both parameters).
You can specify the extrinsics via a config YAML file with the keys `extrinsic_imu2base_quat_xyzw_xyz` and `extrinsic_lidar2base_quat_xyzw_xyz`.

You can dump a config with all the options set to default values by running `rko_lio --dump_config`.
Modify as you require, and pass this file to `rko_lio` using the `-c` flag.
Please check `python/config` in the GitHub repository for example configurations.

An example invocation would then be

```bash
# the config should have the sensor extrinsics if the rosbag doesn't
rko_lio -v -c config.yaml --imu imu_topic --lidar lidar_topic /path/to/rosbag_folder
```

For all possible CLI flags, please check `rko_lio --help`.

</details>

For more install and usage instructions of our python interface, please refer to the [python readme](python#rko_lio---python-bindings) and the [config doc](/docs/config.md).

The python interface to our system can be convenient to investigate recorded data offline as you don't need to setup a ROS environment first.

<details>
<summary><b>But please prefer the ROS version over the python version if you can!</b></summary>
<br />

The ROS version is the intended way to use our odometry system on a robot.
The ROS version also has better performance mainly due to how we read incoming data.
Without getting into details, if you can, you should prefer using the ROS version.
For offline use, we provide a way to directly inspect and run our odometry on recorded rosbags (see offline mode in [ROS usage](ros#usage)), which should be preferred over the python dataloader.
The python interface is merely meant to be a convenience.

</details>

### ROS2

> We are working on getting the odometry package into the ROS index, so you can install it using system package managers instead of building from source.

<details>
<summary><b>Here's a ROS2 quick start!</b></summary>
<br />

Clone the repository into your ROS workspace and then

```bash
# we use ninja to build by default
colcon build --packages-select rko_lio # --symlink-install --event-handlers console_direct+
```

To launch the odometry node:

```bash
ros2 launch rko_lio odometry.launch.py # config_file:=/path/to/a/config.yaml rviz:=true
```

Note that we have some [default build configuration options](ros/colcon.pkg) which should automatically get picked up by colcon.
We have a few dependencies, but as long as these defaults apply, the package should build without any further consideration.
If you encounter any issues, please check [docs/build.md](docs/build.md) for further details or open an issue afterwards.

</details>

Please refer to the [ROS readme](ros) for further ROS-specific details.

## About

RKO_LIO is a LiDAR-inertial odometry system that is by design simple to deploy on different sensor configurations and robotic platforms with as minimal a change in configuration as necessary.

We have no restriction on which LiDAR you can use, and you can do so without changing any config (we've tested Velodyne, Ouster, Hesai, Livox, Robosense, Aeva sensors).
For using an IMU, we require only the accelerometer and gyroscope readings, the bare minimum.
You don't need to look up manufacturer spec sheets to provide noise specifications, etc.

All you need to provide is the extrinsic transformation between the IMU and LiDAR and you can start using our system for your LiDAR-inertial odometry needs!

For a brief demo showcasing the capabilities of our odometry on data from different platforms, click the video below:

[![Thumbnail](https://img.youtube.com/vi/QrVHip5opIc/maxresdefault.jpg)](https://www.youtube.com/watch?v=QrVHip5opIc)

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
