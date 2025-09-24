<h1 align="center">
  RKO LIO
</h1>
<h3 align="center">Robust LiDAR-Inertial Odometry Without Sensor-Specific Modelling</h3>

<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2509.06593-b31b1b.svg)](https://arxiv.org/abs/2509.06593) [![GitHub License](https://img.shields.io/github/license/PRBonn/rko_lio)](/LICENSE) [![GitHub last commit](https://img.shields.io/github/last-commit/PRBonn/rko_lio)](/) [![PyPI - Version](https://img.shields.io/pypi/v/rko_lio?color=blue)](https://pypi.org/project/rko-lio/)


<p align="center">
  ROS Distros:<br />
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_humble.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/ros_build_humble.yaml?branch=master&label=Humble" alt="Humble" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_jazzy.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/ros_build_jazzy.yaml?branch=master&label=Jazzy" alt="Jazzy" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_kilted.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/ros_build_kilted.yaml?branch=master&label=Kilted" alt="Kilted" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build_rolling.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/ros_build_rolling.yaml?branch=master&label=Rolling" alt="Rolling" /></a>
</p>

</div>

<p align="center">
  Python Bindings:<br />
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2204.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_ubuntu_2204.yaml?branch=master&label=Ubuntu%2022.04" alt="Ubuntu 22.04" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2204_arm.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_ubuntu_2204_arm.yaml?branch=master&label=Ubuntu%2022.04%20ARM" alt="Ubuntu 22.04 ARM" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2404.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_ubuntu_2404.yaml?branch=master&label=Ubuntu%2024.04" alt="Ubuntu 24.04" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_ubuntu_2404_arm.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_ubuntu_2404_arm.yaml?branch=master&label=Ubuntu%2024.04%20ARM" alt="Ubuntu 24.04 ARM" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_macos_14.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_macos_14.yaml?branch=master&label=macOS%2014" alt="macOS 14" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_macos_15.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_macos_15.yaml?branch=master&label=macOS%2015" alt="macOS 15" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_windows_2022.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_windows_2022.yaml?branch=master&label=Windows%202022" alt="Windows 2022" /></a>
<a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings_windows_11_arm.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings_windows_11_arm.yaml?branch=master&label=Windows%2011%20ARM" alt="Windows 11 ARM" /></a>
</p>


<p align="center">
  <a href="https://www.youtube.com/watch?v=NNpzXdf9XmU" target="_blank">
    <img src="/docs/example_multiple_platforms_shadow.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" style="max-width: 100%; height: auto;" width="800"/>
  </a>
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

## Quick Start

[demo video here]

Assuming you have a rosbag (ros1/ros2) which contains a TF tree, you can inspect the results of our odometry system with the following two steps

```bash
pip install rko_lio rosbags rerun-sdk
```

Why these three packages?
- `rko_lio` -> our odometry package
- `rosbags` -> required for our rosbag dataloader. Both ros1 and ros2 bags are supported!
- `rerun-sdk` -> required for our optional visualizer (`-v` flag)

Next, run

```bash
# data path should be a directory with *.bag files (ROS1) or a metadata.yaml (ROS2)
rko_lio -v /path/to/data
```

and you should be good to go!

In case you're interested in running the odometry on a robot, please use the [ROS version](/ros/README.md).

<details>
<summary><b>Click here for some more details on how the above works and how to use RKO LIO!</b></summary>
<br />

You can specify a dataloader to use with `-d`, but if you don't, we try to guess the format based on the layout of the data.

Our rosbag dataloader works with either ros1 or ros2 bags. Place split ros1 bags in a single folder and pass the folder as the data path. ros2 bags will need a `metadata.yaml` file in the folder. Note that we don't support running RKO LIO on partial or incomplete bags.

By default, we assume there is just one IMU topic and one LiDAR topic in the bag, in which case we automatically pick up the topic names and proceed further. If there are multiple topics per sensor, you will be prompted to select one via the `--imu` or `--lidar` flags, which you can pass to `rko_lio`.

Next, we assume there is a (static) TF tree in the bag. If so, we take the frame ids from the message topics we just picked up, build a static TF tree, and then query it for the extrinsic from IMU to LiDAR. Our odometry estimates the robot pose with respect to a base frame, and by default, we assume the LiDAR frame to be the base frame. If you would like to use a different frame, you can pass the frame id with `--base_frame` (note the other options available with `--help`). The TF tree will be queried for the appropriate transformations (if they exist in the bag!).

In case there is no TF tree in the bag, then you will have to manually specify the extrinsics for IMU to base frame and LiDAR to base frame, as these two are **required** parameters. Set one of the extrinsics to identity if you want that one to be the base frame (you will still have to specify both parameters). You can specify the extrinsics via a config YAML file with the keys `extrinsic_imu2base_quat_xyzw_xyz` and `extrinsic_lidar2base_quat_xyzw_xyz`.

You can dump a config with all the options set to default values by running `rko_lio --dump_config`. Modify as you require, and pass this file to `rko_lio` using the `-c` flag. Please check `python/config` in the GitHub repository for example configurations.

An example invocation would then be

```bash
# the config should have the sensor extrinsics if the rosbag doesn't
rko_lio -v -c config.yaml --imu imu_topic --lidar lidar_topic /path/to/rosbag_folder
```

For all possible CLI flags, please check `rko_lio --help`.

</details>

<details>
<summary><b>Please prefer the ROS version over the python version if you can!</b></summary>
<br />

The [ROS version](/ros/README.md) is the intended way to use our odometry system on a robot. The ROS version also has better performance mainly due to how we read incoming data. Without getting into details, if you can, you should prefer using the ROS version. For offline use, we provide a way to directly inspect and run our odometry on recorded rosbags (see offline mode in [ROS usage](/ros/README.md#usage)), which should be preferred over the python dataloader. The python interface is merely meant to be a convenience.

</details>

## About

RKO LIO is a LiDAR-inertial odometry system that is by design simple to deploy on different sensor configurations and robotic platforms with as minimal a change in configuration as necessary.

We have no restriction on which LiDAR you can use, and you can do so without changing any config (we've tested Velodyne, Ouster, Hesai, Livox, Robosense, Aeva sensors).
For using an IMU, we require only the accelerometer and gyroscope readings, the bare minimum.
You don't need to look up manufacturer spec sheets to provide noise specifications, etc.

All you need to provide is the extrinsic transformation between the IMU and LiDAR and you can start using our system for your LiDAR-inertial odometry needs!

For a brief demo of our odometry on data from different platforms, click below for a (YouTube) video:

<a href="https://www.youtube.com/watch?v=NNpzXdf9XmU" target="_blank">
  <img src="/docs/odometry_video_thumbnail.png" alt="Odometry video thumbnail" />
</a>


## A note on transformations

It bears mentioning here our convention for specifying sensor extrinsics, the one parameter we do require you to provide. Throughout this package, we refer to transformations using `transform_<from-frame>_to_<to-frame>` or `transform_<from-frame>2<to-frame>`. By this, we mean a transformation that converts a vector expressed in the `<from-frame>` coordinate system to the `<to-frame>` coordinate system.

Mathematically, this translates to:

$$
\mathbf{v}^{\text{to}} = {}^{\text{to}} \mathbf{T}_{\text{from}}  \mathbf{v}^{\text{from}}
$$

The superscript on the vector indicates the frame in which the vector is expressed, and $${}^{\text{to}} \mathbf{T}_{\text{from}}$$ corresponds to `transform_<from-frame>_to_<to-frame>`.

## Platforms and Sensors Tested

RKO LIO has been tested on a variety of platforms with different sensor setups:

- Car: Ouster OS1-128; OS2-128, Livox Avia, Aeva Aeries II (HeLiPR dataset)
- Backpack: Hesai XT32, QT32, QT64 (DigiForests dataset); QT128
- Forestry Harvester: Hesai XT32
- Quadruped: Velodyne VLP-16 (Leg-KILO dataset)
- Bicycle: Livox Avia (thanks to @rlabs-oss, [YouTube video](https://www.youtube.com/watch?v=dKDGIAu628w))

If you've tested RKO LIO on any other platform or sensor configuration, I'd be glad to list it here.
Please reach out by [email](mailto:rm.meher97@gmail.com) or open a PR!

## License

This project is free software made available under the MIT license. For details, see the [LICENSE](LICENSE) file.

## Citation

If you found this work useful, please consider leaving a star on this repository and citing our [paper](https://arxiv.org/abs/2509.06593):

```bib
@article{malladi2025arxiv,
  author      = {M.V.R. Malladi and T. Guadagnino and L. Lobefaro and C. Stachniss},
  title       = {A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling},
  journal     = {arXiv preprint},
  year        = {2025},
  volume      = {arXiv:2509.06593},
  url         = {https://arxiv.org/pdf/2509.06593},
}
```

### RA-L Submission

You can check out the branch `ral_submission` for the version of the code used for submission to RA-L. Please note that that branch is meant to be an as-is reproduction of the code used during submission and is not supported. The `master` and release versions are vastly improved, supported, and are the recommended way to use this system.

## Acknowledgements

<details>
<summary>KISS-ICP, Kinematic-ICP, Bonxai, PlotJuggler, Rerun</summary>

This package is inspired by and would not be possible without the work of [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp).
Additionally, we use and rely heavily on, either in the package itself or during development, [Bonxai](https://github.com/facontidavide/Bonxai), [PlotJuggler](https://github.com/facontidavide/PlotJuggler), [Rerun](https://github.com/rerun-io/rerun), and of course ROS itself.

A special mention goes out to [Rerun](https://rerun.io/) for providing an extremely easy-to-use but highly performative visualization system. Without this, I probably would not have made a python interface at all.

</details>
