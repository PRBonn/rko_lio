ROS
===

.. raw:: html

  <div>
  <p>
    ROS Distros:
  <a href="https://github.com/PRBonn/rko_lio/actions/workflows/ros_build.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/ros_build.yaml?branch=master&label=ROS%20Build" alt="ROS Build" /></a>
  </p>
  </div>

Supported distros: Humble, Jazzy, Kilted, Rolling.

.. contents::
   :local:
   :depth: 2

Setup
-----

Install pre-built releases
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    sudo apt install ros-${ROS_DISTRO}-rko-lio

Build from source
^^^^^^^^^^^^^^^^^

The system dependencies are:

- CMake, a ROS environment.
- Optionally: Eigen, Sophus, nlohmann_json, TBB. Bonxai is always fetched (no system release exists).

Clone the repository into a colcon workspace's ``src``.

You can handle dependencies through either:

- ``rosdep``
- CMake ``FetchContent``

For ``rosdep``, run the following in your workspace directory:

.. code-block:: bash

   rosdep install --from-paths src --ignore-src -r -y

For ``FetchContent``, enable ``RKO_LIO_FETCH_CONTENT_DEPS`` when building.

I provide certain default colcon CMake arguments in ``colcon.pkg`` which you **might not want**:

- ``CMAKE_BUILD_TYPE=Release`` -- self-explanatory.
- ``RKO_LIO_BUILD_ROS=ON`` -- only build the ROS wrapper when enabled.
- ``CMAKE_EXPORT_COMPILE_COMMANDS=ON`` -- useful for development.

These flags are picked up automatically when you build the package with colcon.
Edit ``colcon.pkg`` to disable any flag.

From the workspace folder:

.. code-block:: bash

   colcon build --packages-select rko_lio  # --symlink-install --event-handlers console_direct+

If you didn't choose ``rosdep`` to setup dependencies, run instead:

.. code-block:: bash

   colcon build --packages-select rko_lio --cmake-args -DRKO_LIO_FETCH_CONTENT_DEPS=ON

Also consider using ``Ninja`` as a generator to speed up builds instead of Make.
If you do use Make, make sure (pun intended) to parallelize the build.

CMake notes
^^^^^^^^^^^

I recommend ``cmake>=3.28`` because of how I handle core library dependencies.
That is the default CMake version on Ubuntu 24.04 (also the ROS Jazzy/Kilted target Ubuntu).

Building with ``cmake>=3.22`` is still supported (the default on Ubuntu 22.04, used for Humble) but is less tested.
If you hit build problems on older CMake, please open an issue.

If your distro doesn't ship a recent enough CMake and you'd like to upgrade:

.. code-block:: bash

   export CMAKE_VERSION="3.28.6"
   cd /tmp
   wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz
   tar -zxf cmake-${CMAKE_VERSION}.tar.gz
   rm cmake-${CMAKE_VERSION}.tar.gz
   cd cmake-${CMAKE_VERSION}
   ./bootstrap --system-curl --prefix=/usr/local
   make -j$(nproc)
   sudo make install
   cd /tmp
   rm -rf cmake-${CMAKE_VERSION}
   cmake --version

Usage
-----

Everything is launched through ``odometry.launch.py``.
At minimum, set ``lidar_topic``, ``imu_topic`` and ``base_frame``:

.. code-block:: bash

   ros2 launch rko_lio odometry.launch.py \
       lidar_topic:=/your/lidar imu_topic:=/your/imu base_frame:=base

To see all parameters with their descriptions:

.. code-block:: bash

   ros2 launch rko_lio odometry.launch.py -s

For details on the LIO parameters and the data assumptions, see :doc:`config` and :doc:`data`.
ROS-specific behaviour is described below.

You can put all parameters into a YAML config and pass it with ``config_file:=/path``.
The path is used as-is.
``config/ros_default.yaml`` lists the defaults explicitly with placeholders for the topic / frame names.
Parameters specified on the CLI override values from the config file.

Enable RViz with ``rviz:=true``; it loads ``config/default.rviz`` by default and patches the fixed/target frames from your ``base_frame`` / ``odom_frame``.

By default the launch file runs the threaded online node and publishes lidar-rate odometry.
Pass ``mode:=offline`` to drain a rosbag at full speed (see :ref:`offline-mode`), or ``odom_at_imu_rate:=true`` to additionally publish IMU-rate odometry alongside the lidar-rate output (see :ref:`imu-rate-odometry`).

A typical invocation:

.. code-block:: bash

   ros2 launch rko_lio odometry.launch.py \
       config_file:=/path/to/config/file \
       rviz:=true

Frames and extrinsics
^^^^^^^^^^^^^^^^^^^^^

If your TF tree is well defined -- it exists, and the message ``frame_id`` of every topic matches a frame in the TF tree (I've seen both conditions fail) -- the sensor frames are picked up from the topics and the extrinsics resolved via TF lookup.
You don't need to set anything else.

Otherwise, override the frame ids with ``imu_frame`` / ``lidar_frame``, or specify the extrinsics directly in a config file:

.. code-block:: yaml

   # only valid in a config file, not on the CLI
   extrinsic_imu2base_quat_xyzw_xyz:   [0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0]
   extrinsic_lidar2base_quat_xyzw_xyz: [0.0, 0.0, 0.0, 1.0,  0.1, 0.0, 0.05]

The format is ``[qx, qy, qz, qw, x, y, z]``, with the convention described under :ref:`data-extrinsics-convention`.
If one of the two extrinsics is set, both must be set; pick identity for whichever frame you want to treat as the base.

But really, if you have a TF problem, just fix it instead.

If your platform starts at rest, also set ``initialization_phase:=true``.
The system uses the IMU between the first two scans to initialise roll / pitch and IMU biases, which makes a noticeable difference -- especially when starting on an inclined surface.


Published topics
^^^^^^^^^^^^^^^^

- ``rko_lio/odom`` -- ``nav_msgs/Odometry``, always published at LiDAR rate.

  Renamable via ``odom_topic``.
  The twist is in ``base_frame`` coordinates and estimated from scan registration.
  A TF ``base_frame`` -> ``odom_frame`` is also broadcast alongside this topic; see ``invert_odom_tf`` if your TF tree needs the opposite parent/child relationship.

- ``rko_lio/frame`` -- ``sensor_msgs/PointCloud2``, only published if ``publish_deskewed_scan:=true``.

  The input scan after deskewing.
  Renamable via ``deskewed_scan_topic``.

- ``rko_lio/local_map`` -- ``sensor_msgs/PointCloud2``, only published if ``publish_local_map:=true``.

  The local map maintained by the odometry, re-published every ``publish_map_after`` seconds.
  Renamable via ``map_topic``.

- ``rko_lio/lidar_acceleration`` -- ``geometry_msgs/AccelStamped``, only published if ``publish_lidar_acceleration:=true``.

  Linear acceleration of ``base_frame`` expressed in ``base_frame`` coordinates.
  Quite noisy, since it is essentially a double time derivative of the registration-based pose update.
  The topic name is fixed.

- ``rko_lio/odom_at_imu_rate`` -- ``nav_msgs/Odometry``, only published when running with ``odom_at_imu_rate:=true``.

  IMU-rate odometry; see :ref:`imu-rate-odometry` below.
  Renamable via ``seq.odom_at_imu_rate_topic``.

- ``rko_lio/bag_progress`` -- only published in ``mode:=offline``.

  Two values: percentage of the bag processed and an ETA.

.. _imu-rate-odometry:

IMU-rate odometry
^^^^^^^^^^^^^^^^^

By default the launch file publishes odometry at the LiDAR scan rate (typically ~10 Hz).
To additionally publish odometry at IMU rate, pass ``odom_at_imu_rate:=true`` (only meaningful with ``mode:=online``; the launch file warns and falls back if combined with ``mode:=offline``).

What it actually publishes: the core LIO maintains a full SE(3) pose plus linear velocity that is forward-integrated from the IMU on top of the latest LiDAR-aligned pose, and reset to the optimised pose after every successful scan registration.
The IMU callback publishes that integrated state on ``seq.odom_at_imu_rate_topic`` (default ``rko_lio/odom_at_imu_rate``).
The lidar-rate odometry topic still ticks at the scan rate as usual.

The ``base_frame`` -> ``odom_frame`` TF stays at LiDAR rate by default.
Set ``seq.tf_at_imu_rate:=true`` to broadcast it at IMU rate instead.

Two caveats are worth mentioning:

- This pipeline relies on the IMU and LiDAR streams being well synchronised. If the topics drift out of sync, the IMU-rate output will break. The standard LiDAR-rate odometry (``odom_at_imu_rate:=false``, the default) is more tolerant of timing slop -- prefer it unless you specifically need the higher-rate output.

- The IMU-rate output is raw IMU integration between scan resets and will be jerky. For most downstream uses you want to smooth it, using say an EKF (I haven't tried it myself but `robot_localization <https://index.ros.org/p/robot_localization/>`_ should have one). The same can also be useful on the LiDAR-rate output if you find that a bit jerky as well.

.. _offline-mode:

Offline mode
^^^^^^^^^^^^

Set ``mode:=offline`` to drain a rosbag directly instead of subscribing to live topics.
This is faster than ``ros2 bag play`` since it doesn't have to honour wall-clock pacing.

Pass ``bag_path:=`` pointing at the bag directory (with ``.db3`` / ``.mcap`` and the appropriate plugins installed).
The offline node also publishes ``rko_lio/bag_progress`` so you can monitor how far through the bag it has gotten -- two values, percentage and ETA.

A typical invocation:

.. code-block:: bash

   ros2 launch rko_lio odometry.launch.py \
       config_file:=/path/to/config/file \
       rviz:=true \
       mode:=offline \
       bag_path:=/path/to/rosbag/directory

If your LiDAR per-point timestamps confuse the heuristic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If startup throws ``Runtime Error: TimestampProcessingConfig does not cover this particular case of data``, see :ref:`Configuring -> LiDAR per-point timestamps <config-lidar-timestamps>` for the override knobs.
The ROS parameters are ``lidar_timestamps.multiplier_to_seconds`` / ``lidar_timestamps.force_absolute`` / ``lidar_timestamps.force_relative``.

References
----------

- :doc:`../generated/index` -- C++ API (Doxygen).
- :doc:`../__PACKAGE` -- ``package.xml``.

.. toctree::
   :hidden:

   ../generated/index
   ../__PACKAGE
