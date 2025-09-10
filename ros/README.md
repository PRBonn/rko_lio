# RKO_LIO - LiDAR-Inertial Odometry

We support ROS2 Humble, Jazzy and Kilted.
Support for rolling is planned.

## Setup

### Build from source

Our system dependencies are:
- CMake, ROS environment
- Optionally: Ninja, Eigen, Sophus, nlohmann_json, TBB, Bonxai (please see [build.md](../docs/build.md) for more details)

Clone the repository into a colcon workspace's `src`. From the workspace folder, you can then

```bash
colcon build --packages-select rko_lio # --symlink-install --event-handlers console_direct+
```

We provide a `colcon.pkg` [file](colcon.pkg) which defines default values for some CMake arguments. Details from [build.md](../docs/build.md) apply here, but most importantly note that we use `ninja` as a generator by default (you might need to install it, or change it to something else), and we have `RKO_LIO_FETCH_CONTENT_DEPS=ON` by default. This last option handles our optional dependencies automatically.

In case you'd like to use a different generator, or provide our optional dependencies yourself, either modify `colcon.pkg` or override those CMake flags when invoking colcon, for example, by

```bash
colcon build --packages-select rko_lio --cmake-args -DRKO_LIO_FETCH_CONTENT_DEPS=OFF # --event-handlers console_direct+
```

## Usage

We provide an online node, an online node component (see [here](rko_lio/CMakeLists.txt#L54)), and an offline node.

The offline node provides a way to directly read data from a rosbag, instead of the usual pattern of playing the bag with `ros2 bag play`.

Both nodes can be launched via the `odometry.launch.py` [launch file](launch/odometry.launch.py), with the different modes being specified by the `mode` argument (the default is `online`).

Check all available configuration options for the launch file by running

```bash
ros2 launch rko_lio odometry.launch.py -s
```

That will also provide additional documentation about the different parameters. For some additional details regarding the odometry parameters, please refer to [config.md](../docs/config.md). ROS-specific parameters are covered here.

At minimum, you'll need to specify the `lidar_topic`, `imu_topic` and `base_frame` parameters.

If your TF tree is well defined, i.e., it exists and the message frame ids match the frame ids in the TF tree (i've seen both conditions fail), then the frame ids are picked up from the topics and the extrinsics via TF lookup.
Otherwise you'll need to either specify just the frame ids (if there's a mismatch), or specify the extrinsics via `extrinsic_lidar2base_quat_xyzw_xyz` written as quaternion (xyzw) and translation (xyz) in a list.
Similarly for the IMU to base.
But really, if you have a TF problem, just fix it instead.

`config/default.yaml` specifies the default set of parameters explicitly, and also leaves some placeholders you can modify to pass the `lidar_topic` and similar.

Please note that the parameter definitions in a config file take priority over those defined from the CLI. This behaviour is planned to be fixed (PR welcome).

As mentioned before regarding the offline node, you can use it to read a bag directly and run the odometry on it at the same time. Pass the `bag_filename:=` parameter to the launch file (which should be a folder containing the .db3 or .mcap or other ROS supported formats).

You can enable rviz visualization by passing `rviz:=true` which launches an rviz window simultaneously using the default rviz config file in `config/default.rviz`.

An example full invocation for running the offline node with rviz can look like this

```bash
ros2 launch rko_lio odometry.launch.py \
    mode:=offline \
    config_file:=/path/to/config/file \
    bag_filename:=/path/to/rosbag/directory \
    rviz:=true
```

### Published topics

- `/rko_lio/odom`: Odometry topic, the name can be modified using the `odom_topic` parameter. This also includes the twist of the `base_frame` expressed in `base_frame` coordinates. This twist is estimated from the lidar scan registration. A TF is also simultaneously published from the `base_frame` to the `odom_frame`. Please note the parameter `invert_odom_tf` in case your TF configuration requires this (you're running multiple odometries or some other complicated setup).
- `/rko_lio/frame`: The input lidar scan deskewed using the IMU data. Only published if `publish_deskewed_cloud:=true`.
- `/rko_lio/local_map`: The local map the odometry maintains is published at a set frequency given by `publish_map_after` (seconds), and only if `publish_local_map:=true`.
- `/rko_lio/acceleration`: Linear acceleration of the `base_frame` expressed in `base_frame` coordinates. Note that this acceleration can be quite noisy, as it is essentially a double time derivative of the pose update from the lidar scan registration (similar to the twist/velocity).

The offline node additionally publishes a convenient topic `rko_lio/bag_progress` which you can use to monitor bag playing progress. It has two values, a percentage completion and an ETA.
