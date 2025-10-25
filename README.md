<h1 align="center">
  RKO-LIO
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
    <img src="/docs/_static/example_multiple_platforms_shadow.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" style="max-width: 100%; height: auto;" width="800"/>
  </a>
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

## Quick Start

<!-- [demo video here] -->

The following is the minimal information you need to get going. For more details, please check the [docs](https://prbonn.github.io/rko_lio/).

Assuming you have a rosbag (ros1/ros2) which contains a TF tree, you can run RKO-LIO through

```bash
pip install rko_lio rosbags rerun-sdk
# data path should be a directory with *.bag files (ROS1) or a metadata.yaml (ROS2)
rko_lio -v /path/to/data
```

Why `pip` install those three packages?
- `rko_lio` -> our odometry package
- `rosbags` -> required for our rosbag dataloader. Both ros1 and ros2 bags are supported!
- `rerun-sdk` -> required for our optional visualizer (`-v` flag)

Check further options for the CLI through `rko_lio --help`.

### ROS

Supported distros: Humble, Jazzy, Kilted, Rolling.

```bash
sudo apt install ros-$ROS_DISTRO-rko-lio
ros2 launch rko_lio odometry.launch.py imu_topic:=<topic> lidar_topic:=<topic> base_frame:=base_link
```

The three parameters above are the minimum you need to specify for the launch file.

Check further launch configuration options through `ros2 launch rko_lio odometry.launch.py -s`

## License

This project is free software made available under the MIT license. For details, see the [LICENSE](LICENSE) file.

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

### RA-L Submission

You can check out the branch `ral_submission` for the version of the code used for submission to RA-L. Please note that that branch is meant to be an as-is reproduction of the code used during submission and is not supported. The `master` and release versions are vastly improved, supported, and are the recommended way to use this system.

## Acknowledgements

<details>
<summary>KISS-ICP, Kinematic-ICP, Bonxai, PlotJuggler, Rerun</summary>

This package is inspired by and would not be possible without the work of [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp).
Additionally, we use and rely heavily on, either in the package itself or during development, [Bonxai](https://github.com/facontidavide/Bonxai), [PlotJuggler](https://github.com/facontidavide/PlotJuggler), [Rerun](https://github.com/rerun-io/rerun), and of course ROS itself.

A special mention goes out to [Rerun](https://rerun.io/) for providing an extremely easy-to-use but highly performative visualization system. Without this, I probably would not have made a python interface at all.

</details>
