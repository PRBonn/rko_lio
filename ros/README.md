# RKO LIO - LiDAR-Inertial Odometry

Supported ROS Distros: Humble, Jazzy, Kilted and Rolling.

## Setup

### Build from source

Our system dependencies are:
- CMake, ROS environment
- Optionally: Eigen, Sophus, nlohmann_json, TBB, Bonxai (please see [build.md](../docs/build.md) for more details)

Clone the repository into a colcon workspace's `src`. From the workspace folder, you can then

```bash
colcon build --packages-select rko_lio # --symlink-install --event-handlers console_direct+
```

We provide a `colcon.pkg` [file](colcon.pkg) which defines a default CMake configuration.
Details from [build.md](../docs/build.md) apply here, but most importantly we have `CMAKE_BUILD_TYPE=Release` and `RKO_LIO_FETCH_CONTENT_DEPS=ON` by default.
The last option handles our optional dependencies automatically.

In case you'd like to change the build type or provide our optional dependencies yourself, please modify `colcon.pkg` manually. 

> I'd earlier assumed one could override colcon.pkg arguments via --cmake-args when invoking colcon, but this doesn't seem to be the case. If anyone knows a fix, please open an issue or PR!

Also consider using `Ninja` as a generator to speed up builds instead of `Make`.
If you do use Make, make sure (pun intended) to parallelize the build.

## Usage

We provide an online node component, a standalone online node and an offline node.

The offline node provides a way to directly read data from a rosbag, instead of the usual pattern of playing the bag with `ros2 bag play`.

Both nodes can be launched via the `odometry.launch.py` [launch file](launch/odometry.launch.py), with the different modes being specified by the `mode` argument (the default is `online`).

Check all available configuration options for the launch file by running

```bash
ros2 launch rko_lio odometry.launch.py -s
```

That will also provide additional documentation about the different parameters.
For some additional details regarding the odometry parameters and data itself, please refer to [config.md](../docs/config.md) and [data.md](../docs/data.md).
ROS-specific parameters are covered here.

At minimum, you'll need to specify the `lidar_topic`, `imu_topic` and `base_frame` parameters.

You can define all parameters in a config file and pass it with the launch argument `config_file:=/path`.
Please note that we don't modify the path you provide in any way.

If your TF tree is well defined, i.e., it exists and the message frame ids match the frame ids in the TF tree (i've seen both conditions fail), then the sensor frame ids are picked up from the topics and the extrinsics via TF lookup.
Otherwise you'll need to either specify just the frame ids (if there's a mismatch), or specify the extrinsics via `extrinsic_lidar2base_quat_xyzw_xyz` written as quaternion (xyzw) and translation (xyz) in a list (only supported via a config file).
Similarly for the IMU to base as `extrinsic_imu2base_quat_xyzw_xyz`.
But really, if you have a TF problem, just fix it instead.

`config/default.yaml` specifies the default set of parameters explicitly, and also leaves some placeholders you can modify to pass the `lidar_topic` and similar.

Please note that the parameter definitions from the CLI will override those provided in a config file.

You can enable rviz visualization by passing `rviz:=true` which launches an rviz window simultaneously using the default rviz config file in `config/default.rviz`.

An example full invocation with rviz can look like this

```bash
ros2 launch rko_lio odometry.launch.py \
    config_file:=/path/to/config/file \
    rviz:=true
```

### Published topics

- `/rko_lio/odometry`: Odometry topic, the name can be modified using the `odom_topic` parameter. This also includes the twist of the `base_frame` expressed in `base_frame` coordinates. This twist is estimated from the lidar scan registration. A TF is also simultaneously published from the `base_frame` to the `odom_frame`. Please note the parameter `invert_odom_tf` in case your TF configuration requires this (you're running multiple odometries or some other complicated setup).
- `/rko_lio/frame`: The input lidar scan deskewed using the IMU data. Only published if `publish_deskewed_scan:=true`.
- `/rko_lio/local_map`: The local map the odometry maintains is published at a set frequency given by `publish_map_after` (seconds), and only if `publish_local_map:=true`.
- `/rko_lio/linear_acceleration`: Linear acceleration of the `base_frame` expressed in `base_frame` coordinates. Note that this acceleration can be quite noisy, as it is essentially a double time derivative of the pose update from the lidar scan registration (similar to the twist/velocity). Only published if `publish_lidar_acceleration:=true`.

### Offline node

As mentioned before, you can use the offline node to read a bag directly and run the odometry on it at the same time.

Specify `mode:=offline` as the default is `online`.

Pass the `bag_path:=` parameter to the launch file, which should be a folder containing the .db3 or .mcap or other ROS supported formats (you probably need the respective plugins).

The offline node additionally publishes a convenient topic `/rko_lio/bag_progress` which you can use to monitor bag playing progress.
It has two values, a percentage completion and an ETA.

An example invocation would then look like

```bash
ros2 launch rko_lio odometry.launch.py \
    config_file:=/path/to/config/file \
    rviz:=true \
    mode:=offline \
    bag_path:=/path/to/rosbag/directory
```
