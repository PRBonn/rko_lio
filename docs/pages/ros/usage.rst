ROS - Usage
===========

I provide an online node component, a standalone online node, and an offline node.

The offline node provides a way to directly read data from a rosbag, instead of the usual pattern of playing the bag with ``ros2 bag play``.

Both nodes can be launched via the ``odometry.launch.py`` launch file, specifying the ``mode`` argument (default: ``online``).

To see all possible configuration options:

.. code-block:: bash

   ros2 launch rko_lio odometry.launch.py -s

That will also provide additional documentation about the different parameters.

For some additional details regarding the odometry parameters and data itself, please refer to :doc:`../config` and :doc:`../data`.
ROS-specific parameters are covered here.

At minimum, you'll need to specify the ``lidar_topic``, ``imu_topic`` and ``base_frame`` parameters.

You can define all parameters in a config file and pass it with the launch argument ``config_file:=/path``.
Please note that we don't modify the path you provide in any way.

If your TF tree is well defined, i.e., it exists and the message frame ids match the frame ids in the TF tree (I've seen both conditions fail), then the sensor frame ids are picked up from the topics and the extrinsics via TF lookup.

Otherwise you'll need to either specify just the frame ids (if there's a mismatch), or specify the extrinsics via ``extrinsic_lidar2base_quat_xyzw_xyz`` written as quaternion (xyzw) and translation (xyz) in a list (only supported via a config file).
Similarly for the IMU to base as ``extrinsic_imu2base_quat_xyzw_xyz``.

But really, if you have a TF problem, just fix it instead.

``config/default.yaml`` specifies the default set of parameters explicitly, and also leaves some placeholders you can modify to pass the ``lidar_topic`` and similar.

Please note that the parameter definitions from the CLI will override those provided in a config file.

You can enable RViz visualization by passing ``rviz:=true`` which launches an RViz window simultaneously using the default RViz config file in ``config/default.rviz``.

An example full invocation with RViz can look like this:

.. code-block:: bash

   ros2 launch rko_lio odometry.launch.py \
       config_file:=/path/to/config/file \
       rviz:=true

Published topics
----------------

- ``/rko_lio/odometry``: Odometry topic, the name can be modified using the ``odom_topic`` parameter.

  This also includes the twist of the ``base_frame`` expressed in ``base_frame`` coordinates.
  This twist is estimated from the LiDAR scan registration.

  A TF is also simultaneously published from the ``base_frame`` to the ``odom_frame``.
  Please note the parameter ``invert_odom_tf`` in case your TF configuration requires this (you're running multiple odometries or some other complicated setup).

- ``/rko_lio/frame``: The input LiDAR scan deskewed using the IMU data.

  Only published if ``publish_deskewed_scan:=true``.

- ``/rko_lio/local_map``: The local map the odometry maintains is published at a set frequency given by ``publish_map_after`` (seconds), and only if ``publish_local_map:=true``.

- ``/rko_lio/linear_acceleration``: Linear acceleration of the ``base_frame`` expressed in ``base_frame`` coordinates.

  Note that this acceleration can be quite noisy, as it is essentially a double time derivative of the pose update from the LiDAR scan registration (similar to the twist/velocity).

  Only published if ``publish_lidar_acceleration:=true``.

IMU-rate odometry
-----------------

By default the launch file publishes odometry at LiDAR rate. To additionally publish odometry at IMU rate, pass ``odom_at_imu_rate:=true`` (only supported with ``mode:=online``). The launch file then runs the sequential variant ``online_imu_rate_node`` and publishes on the topic given by ``seq.odom_at_imu_rate_topic`` (default ``rko_lio/odom_at_imu_rate``). The ``base_frame`` -> ``odom_frame`` TF is still published at LiDAR rate by default; set ``seq.tf_at_imu_rate:=true`` to shift it to IMU rate as well.

Two caveats are worth mentioning:

- This pipeline relies on the IMU and LiDAR streams being well synchronized. If the topics drift out of sync, the IMU-rate output will break. The standard LiDAR-rate odometry (``odom_at_imu_rate:=false``, the default) is more robust to such issues, so prefer it unless you specifically need the higher-rate output.

- The IMU-rate odometry is raw IMU integration and will be jerky. For most downstream uses, you'll want to run it through a smoother — for example, the EKF in `robot_localization <https://index.ros.org/p/robot_localization/>`_. The same smoothing can also be useful even on the LiDAR-rate odometry.

Offline node
------------

As mentioned before, you can use the offline node to read a bag directly and run the odometry on it at the same time.

Specify ``mode:=offline`` as the default is ``online``.

Pass the ``bag_path:=`` parameter to the launch file, which should be a folder containing the ``.db3`` or ``.mcap`` or other ROS-supported formats (you probably need the respective plugins).

The offline node additionally publishes a convenient topic ``/rko_lio/bag_progress`` which you can use to monitor bag playing progress.
It has two values, a percentage completion and an ETA.

An example invocation would then look like:

.. code-block:: bash

   ros2 launch rko_lio odometry.launch.py \
       config_file:=/path/to/config/file \
       rviz:=true \
       mode:=offline \
       bag_path:=/path/to/rosbag/directory
