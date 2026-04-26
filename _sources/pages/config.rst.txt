Configuring the odometry
========================

This page is a reference for runtime parameters.
The first group -- the LIO core parameters -- is shared between the Python and ROS interfaces.
The remaining groups (extrinsics, per-point timestamp handling, Python pipeline knobs, ROS launch knobs) cover the wrapper-specific bits.

The defaults are sane and I've used them with success across a number of platforms and datasets.
But your specific application might benefit from tuning -- the descriptions below try to make clear when.

Physical units are always SI.

.. contents::
   :local:
   :depth: 2

LIO core parameters
-------------------

These show up under the top level of a Python config and as ROS launch arguments with the same name.

- **deskew** (`bool`, default ``True``)

  Whether to apply scan deskewing before registration.

  This compensates for the motion that happens during LiDAR scan collection, since your platform is probably moving while the LiDAR is collecting data.

  Unless you have very good reason, always keep this enabled.

.. warning::
  ``deskew=True`` **requires** per-point timestamps in the LiDAR scan, i.e., the scan needs to have ``xyzt`` per-point where ``t`` is time.
  If you cannot provide this, then deskewing should be disabled.

- **double_downsample** (`bool`, default ``True``)

  Useful for dense LiDARs.

  Disabling this for sparse sensors, like a VLP-16 compared to an Ouster-128, can potentially improve results.
  Indoor scenes can also see an improvement by disabling this.

- **voxel_size** (`float`, default ``1.0``)

  The voxel resolution of the internal local map (in meters).

  Smaller values create finer maps at higher memory/computation cost.
  Reducing this for sparse sensors or indoor scenes can help improve results.

- **max_range** (`float`, default ``100.0``)

  Maximum usable LiDAR range in meters.

  Points beyond this cutoff are ignored.
  Reducing this is an easy way to reduce compute requirements, since you typically won't need information from 100m away for odometry.
  Nevertheless, this is the default.

- **min_range** (`float`, default ``1.0``)

  Minimum LiDAR range in meters.

  Points closer than this are discarded.
  Useful if your platform shows up in the scan due to occlusions.

- **max_points_per_voxel** (`int`, default ``20``)

  Maximum number of points stored per voxel in the VDB map.

  Affects both memory and ICP data association.

  In case you need more runtime performance, you can reduce this.
  Odometry performance will be a bit affected, but how much depends on the environment.

- **max_correspondance_distance** (`float`, default ``0.5``)

  Maximum distance threshold (meters) for ICP data associations.

- **max_iterations** (`int`, default ``100``)

  Limit on the number of iterations for ICP.

  You can limit this a bit more if runtime is an issue.
  Typically the convergence criterion is satisfied much earlier anyways.

- **convergence_criterion** (`float`, default ``1e-5``)

  Termination criterion for optimization.
  Lower (stricter) values will requires more ICP iterations.

- **max_num_threads** (`int`, default ``0``)

  Only used to parallelize data association for ICP.
  ``0`` means autodetect based on hardware.

  In case compute resources are a constraint, limit this to a few threads and, in order, ``max_points_per_voxel``, ``voxel_size``, ``max_range``, ``max_iterations`` are the parameters you probably care about.

- **initialization_phase** (`bool`, default ``False``)

  Initializes the system orientation (roll and pitch) plus IMU biases using the IMU measurements between the first two LiDAR scans the odometry receives.

  If enabled, the second frame is assumed to be coincident with the first.
  I.e., the assumption is that the system is at rest for that duration and the system is oriented to align with gravity.
  This helps if you start from an inclined surface for example.

  Usually you can leave this enabled. Unless for some reason you need to start the odometry while the system is in motion, then disable this.

  I highly recommend enabling this.

.. warning::
  ``initialization_phase=True`` requires you to ensure that the system starts from rest.
  Otherwise, the system will estimate incorrect biases and the odometry might not work as expected.
  Hence, why this is set to ``False`` by default and is opt-in.

- **max_expected_jerk** (`float`, default ``3.0``)

  This value is used in a Kalman filter to estimate the true body acceleration.

  It should reflect the motion you expect from the platform you will deploy the odometry on.
  A good range is [1-3] m/s³, but it should be fine to leave it at 3 m/s³ as that is a good setting for most platforms.

- **min_beta** (`float`, default ``200.0``)

  The minimum weight applied to an orientation regularization cost during scan alignment.

  Essentially we use the accelerometer readings as an additional observation on the roll and pitch of the system (we need to estimate the true body acceleration using the Kalman filter mentioned above).
  This parameter influences how much importance this additional observation plays in the optimization, as we cannot have a perfect observation of the true body acceleration (the estimate is affected by gravity and the odometry itself).
  The default should be fine for most cases.

  You can set it ``-1`` to disable this additional cost.

Extrinsics
----------

The extrinsic transforms between the IMU / LiDAR and the base frame are specified in YAML as ``[qx, qy, qz, qw, x, y, z]`` lists:

.. code-block:: yaml

   extrinsic_imu2base_quat_xyzw_xyz:   [0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0]
   extrinsic_lidar2base_quat_xyzw_xyz: [0.0, 0.0, 0.0, 1.0,  0.1, 0.0, 0.05]

If one of the two keys is set, both must be set; pick identity for whichever frame you want to treat as the base.
The convention is described under :ref:`data-extrinsics-convention`.

When are these required?

- **ROS**: only if the TF tree isn't well defined or topic ``frame_id``\s don't match the TF tree. With a clean TF tree, the extrinsics are looked up automatically.
- **Python rosbag dataloader**: only if the bag has no static TF tree. With a tree, the extrinsics are pulled from it.
- **Python raw / HeLiPR dataloaders**: always required, supplied via the dataloader's own configuration mechanism (``transforms.yaml`` for raw, etc.).

If you specify the extrinsics in a config but the dataloader / TF tree could also provide them, the config values take priority.

.. _config-lidar-timestamps:

LiDAR per-point timestamps
--------------------------

.. automodule:: rko_lio.config
   :no-index:

The same overrides are surfaced as ROS launch arguments under the ``lidar_timestamps.*`` namespace:

.. code-block:: yaml

   lidar_timestamps.multiplier_to_seconds: 0.0   # 0.0 = autodetect; e.g. 1e-6 for microseconds
   lidar_timestamps.force_absolute: false
   lidar_timestamps.force_relative: false

(They can also be written as a nested mapping ``lidar_timestamps:\n  force_absolute: true`` -- ``launch_ros`` flattens nested dicts before passing them as parameter overrides, so both forms are equivalent.)

Pipeline parameters (Python)
----------------------------

These are passed to the Python ``LIOPipeline`` via the same YAML config as the LIO parameters.

On-disk logging is gated by the CLI flag ``--log`` / ``-l`` (``log_results``, default ``True``).
Without it, ``log_dir``, ``run_name``, and ``dump_deskewed_scans`` have no effect.

- **dump_deskewed_scans** (`bool`, default ``False``)

  Save each deskewed scan to disk under ``log_dir`` as PLY. Useful for debugging the deskewing step or for reuse downstream. Off by default since it generates a lot of data.

- **log_dir** (`Path`, default ``"results"``)

  Where the trajectory file (and dumped scans, if enabled) gets written.

- **run_name** (`str`, default ``"rko_lio_run"``)

  Subdirectory name under ``log_dir``. The run gets an auto-incremented suffix to avoid clobbering previous runs.

Launch parameters (ROS)
-----------------------

These are not part of the LIO algorithm itself; they configure the ROS launch behaviour.
For the full list with descriptions, run ``ros2 launch rko_lio odometry.launch.py -s``.

Mode selection
^^^^^^^^^^^^^^

- **mode** (default ``online``)

  ``online`` subscribes to live topics. ``offline`` drains a rosbag at full speed (also requires ``bag_path``).

- **odom_at_imu_rate** (`bool`, default ``false``)

  When ``true`` and ``mode:=online``, the launch file picks ``online_imu_rate_node``, which additionally publishes IMU-rate odometry. See :doc:`ROS -> IMU-rate odometry <ros>`.

- **bag_path** (offline only, required)

  Path to the bag directory.

- **skip_to_time** (`float`, offline only, default ``0.0``)

  Skip ahead in the bag to this absolute time (seconds) before starting registration.

Topic and frame configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **imu_topic**, **lidar_topic**, **base_frame** (required, via CLI or config)

  The IMU topic, the LiDAR topic, and the base frame of the robot.

- **imu_frame**, **lidar_frame** (optional)

  Override these only if the message ``frame_id`` doesn't match the TF tree.

- **odom_frame** (default ``odom``), **odom_topic** (default ``rko_lio/odom``)

  The odom frame name and the odometry topic name.

- **invert_odom_tf** (`bool`, default ``false``)

  Swap parent / child in the published TF if your topology requires it.

Publishing toggles
^^^^^^^^^^^^^^^^^^

- **publish_local_map** (`bool`, default ``false``), **map_topic** (default ``rko_lio/local_map``), **publish_map_after** (`float`, default ``1.0``)

  Whether to publish the local map, the topic name, and the republish cadence in seconds.

- **publish_deskewed_scan** (`bool`, default ``false``), **deskewed_scan_topic** (default ``rko_lio/frame``)

  Whether to publish the deskewed scan, and the topic name.

- **publish_lidar_acceleration** (`bool`, default ``false``)

  Publish a noisy linear acceleration estimate on ``rko_lio/lidar_acceleration`` (the topic name is fixed, not configurable).

Mode-specific knobs
^^^^^^^^^^^^^^^^^^^

These take effect only in the corresponding mode and otherwise warn that they are being ignored.

- **async.max_lidar_buffer_size** (`int`, default ``50``)

  Threaded path only. Caps the lidar buffer; older frames are dropped past this.

- **seq.odom_at_imu_rate_topic** (default ``rko_lio/odom_at_imu_rate``)

  IMU-rate output topic name.

- **seq.tf_at_imu_rate** (`bool`, default ``false``)

  Broadcast ``base_frame`` -> ``odom_frame`` TF at IMU rate instead of LiDAR rate.

Disk dumping and visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- **dump_results** (`bool`, default ``false``)

  On shutdown, dump the resolved configuration and the full trajectory under ``<results_dir>/<run_name>``. The folder name is auto-incremented to avoid overwrites.

- **results_dir** (default ``results``), **run_name** (default ``rko_lio_odometry_run``)

  Where the dump goes and the subdirectory name within.

- **rviz** (`bool`, default ``false``), **rviz_config_file** (default ``config/default.rviz``)

  Launch RViz alongside the odometry. If you leave the rviz config at the default, the launch file patches it with your ``base_frame`` / ``odom_frame`` and enables ``publish_deskewed_scan`` / ``publish_local_map`` so the visualizer has something to show.

- **log_level** (default ``info``)

  ROS log level.

Other
^^^^^

- **config_file**

  Path to a YAML config that supplies any of the above. CLI values override the file.
