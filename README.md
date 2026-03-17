<h1 align="center">
  RKO-LIO
</h1>
<h3 align="center">Robust LiDAR-Inertial Odometry Without Sensor-Specific Modelling</h3>

<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2509.06593-b31b1b.svg)](https://arxiv.org/abs/2509.06593) [![GitHub License](https://img.shields.io/github/license/PRBonn/rko_lio)](/LICENSE) [![GitHub last commit](https://img.shields.io/github/last-commit/PRBonn/rko_lio)](/)

[![PyPI - Version](https://img.shields.io/pypi/v/rko_lio?color=blue)](https://pypi.org/project/rko-lio/) ![ROS Package Index](https://img.shields.io/ros/v/humble/rko_lio) ![ROS Package Index](https://img.shields.io/ros/v/jazzy/rko_lio) ![ROS Package Index](https://img.shields.io/ros/v/kilted/rko_lio) ![ROS Package Index](https://img.shields.io/ros/v/rolling/rko_lio)

</div>

<p align="center">
  <a href="https://www.youtube.com/watch?v=NNpzXdf9XmU" target="_blank">
    <img src="/docs/_static/example_multiple_platforms_shadow.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" style="max-width: 100%; height: auto;" width="800"/>
  </a>
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

## Quick Start

<!-- [demo video here] -->

Assuming you have a rosbag (ros1/ros2) which contains a TF tree, you can run RKO-LIO through

```bash
pip install rko_lio rosbags rerun-sdk
# data path should be a directory with *.bag files (ROS1) or a metadata.yaml (ROS2)
rko_lio -v /path/to/data
```

Why `pip install` those three packages?
- `rko_lio` -> the odometry package
- `rosbags` -> required for the rosbag dataloader. Both ros1 and ros2 bags are supported!
- `rerun-sdk` -> required for the optional visualizer (`-v` flag)

Check further options for the CLI through `rko_lio --help`.

More details are available in the [Python usage docs](https://prbonn.github.io/rko_lio/pages/python/usage.html).

### Extrinsics and convention

Please note that the system needs the extrinsic to be specified between IMU and LiDAR. Either your data includes this in some format, and then the dataloaders try to automatically read it, or otherwise you can specify it in a config file (required if it's missing in the data). Pass the config file with `rko_lio --config config_file.yaml`. This file needs two keys: `extrinsic_imu2base_quat_xyzw_xyz` and `extrinsic_lidar2base_quat_xyzw_xyz`, which must each be a list. For example, `[0,0,0,1,0,0,0]` for identity. Both keys are required.


Throughout this package, I refer to transformations using `transform_<from-frame>2<to-frame>`. By this, I mean a transformation that converts a vector expressed in the `<from-frame>` coordinate system to the `<to-frame>` coordinate system. Mathematically, this translates to:

$$ \mathbf{v}^{\mathrm{to}} = {}^{\mathrm{to}}\mathbf{T}_{\mathrm{from}} \mathbf{v}^{\mathrm{from}} $$

The superscript on the vector indicates the frame in which the vector is expressed, and $^{ \mathrm{to} }\mathbf{T}_{\mathrm{from}}$ corresponds to `transform_<from-frame>_to_<to-frame>`.

## ROS

Supported distros: Humble, Jazzy, Kilted, Rolling.

```bash
sudo apt install ros-$ROS_DISTRO-rko-lio
```

Or if you'd like to build from source, clone the repo into your colcon workspace and

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rko_lio  # --symlink-install --event-handlers console_direct+
```

In case you cannot system install the necessary dependencies through rosdep, you can also build the dependencies while building RKO-LIO

```bash
colcon build --packages-select rko_lio --cmake-args -DRKO_LIO_FETCH_CONTENT_DEPS=ON
```

A launch file is provided:

```bash
ros2 launch rko_lio odometry.launch.py imu_topic:=<topic> lidar_topic:=<topic> base_frame:=base_link
```

The three parameters `imu_topic`, `lidar_topic`, and `base_frame` are the minimum you need to specify for the launch file.

Check further launch configuration options through `ros2 launch rko_lio odometry.launch.py -s`

More details are available in the [ROS usage docs](https://prbonn.github.io/rko_lio/pages/ros/usage.html).

The same note [above about extrinsics](#extrinsics-and-convention) applies here as well. Though you probably have a well defined TF tree and need not concern yourself with this (I hope).

## Citation

If you found this work useful, please consider leaving a star :star: on this repository and citing our [paper](https://arxiv.org/abs/2509.06593):

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

## Contributing

My goal with this project is to have a simple LiDAR-Inertial odometry system that can work with minimal friction, in the lines of ["it just works"](https://youtu.be/nVqcxarP9J4?si=LvTfV4m0NkNC62Tf&t=6). I gladly welcome any contribution or feedback you think would help in this direction. Performance improvements or bug fixes are of course always appreciated.

Thanks to the following contributors:
<a href="https://github.com/PRBonn/rko_lio/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=PRBonn/rko_lio" />
</a>

## Acknowledgements

<details>
<summary>KISS-ICP, Kinematic-ICP, Bonxai, PlotJuggler, Rerun</summary>

This package is inspired by and would not be possible without the work of [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp).
Additionally, we use and rely heavily on, either in the package itself or during development, [Bonxai](https://github.com/facontidavide/Bonxai), [PlotJuggler](https://github.com/facontidavide/PlotJuggler), [Rerun](https://github.com/rerun-io/rerun), and of course ROS itself.

A special mention goes out to [Rerun](https://rerun.io/) for providing an extremely easy-to-use but highly performative visualization system. Without this, I probably would not have made a python interface at all.

</details>


### RA-L Submission

You can check out the branch `ral_submission` for the version of the code used for submission to RA-L. Please note that that branch is meant to be an as-is reproduction of the code used during submission and is not supported. The `master` and release versions are vastly improved, supported, and are the recommended way to use this system.

## License

This project is free software made available under the MIT license. For details, see the [LICENSE](LICENSE) file.
