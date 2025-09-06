# RKO_LIO - LiDAR-Inertial Odometry Without Sensor-Specific Modelling

RKO_LIO is a LiDAR-inertial odometry system that is by design simple to deploy on different sensor configurations and robotic platforms with as minimal a change in configuration as necessary.

We have no restriction on which LiDAR you can use, and you can do so without changing any config (we've tested Velodyne, Ouster, Hesai, Livox, Robosense, Aeva sensors).
For using an IMU, we require only the accelerometer and gyroscope readings, the bare minimum.
You don't need to look up manufacturer spec sheets to provide noise specifications, etc.

All you need to provide is the extrinsic transformation between the IMU and LiDAR and you can start using our system for your LiDAR-inertial odometry needs!

<p align="center">
  <img src="https://raw.githubusercontent.com/PRBonn/rko_lio/refs/heads/master/docs/example_multiple_platforms.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" />
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

## Quick Start

In case you already have a rosbag which contains a TF tree, you can inspect the results of our odometry system with the following two steps

```bash
pip install rko_lio rosbags rerun-sdk
```

`rko_lio` is our odometry package, `rosbags` is required for using our rosbag dataloader, and `rerun-sdk` is what we use for our optional visualizer.
After everything is installed, run

```bash
rko_lio -v /path/to/rosbag
```

and you should be good to go! For quick details on further options, check `rko_lio --help`.

For detailed install and usage instructions, please refer to the [python bindings readme](python#rko_lio---python-bindings).

## Setup

### ROS2

> We are working on getting the odometry package into the ROS index, so you can install it using system package managers instead of building from source.

We currently support ROS2 Jazzy and Kilted, with plans to additionally support Humble and Rolling.

Clone the repository into your ROS workspace and then

```bash
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

You can install RKO_LIO by simply

```bash
pip install rko_lio
```

We provide wheels for Linux, Mac OS, and Windows.

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
<summary>Please prefer the ROS version over the python version if you can</summary>

**Please note:** the ROS version is the intended way to use our odometry system on a robot.
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
