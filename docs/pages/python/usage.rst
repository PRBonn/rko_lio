Python - Usage
==============

You can pass a number of options to ``rko_lio``.  
For all possible CLI flags, run:

.. code-block:: bash

   rko_lio --help

The ``-v`` flag enables visualization.  

There are three dataloaders available: ``rosbag`` (ROS1 or ROS2), ``raw``, and ``HeLiPR`` (deprecated).  
For most usages, the system will automatically detect which dataloader to use from your data path, but you can choose explicitly with ``-d``.

A config file can be passed with `--config` or `-c`. Dump a config file with all default options using 

.. code-block:: bash

   rko_lio --dump_config

Extrinsic transformations must be specified with the following keys in the config:
- ``extrinsic_imu2base_quat_xyzw_xyz``
- ``extrinsic_lidar2base_quat_xyzw_xyz``

These are **required parameters**. But if the dataloader can provide the extrinsics automatically, see below, then they are **not required**. 

.. warning::
  If your dataloader provides extrinsics, but you specify them in a config, the config values **will** take priority**.

Rosbag Dataloader
-----------------

Our rosbag dataloader works with both ROS1 and ROS2 bags. Place split ROS1 bags together in a single folder and use that folder as the data path.  
ROS2 bags especially require a ``metadata.yaml`` file.

.. note::
  We do not support running RKO LIO on partial or incomplete bags, though you may try, and are encouraged to raise an issue if you require this supported.  

There are certain reasonable defaults:
- The bag contains one IMU topic and one LiDAR topic (otherwise specify with ``--imu`` or ``--lidar`` flags).
- A static TF tree is in the bag. We build a static TF tree from topic frame ids and query it for extrinsic between IMU and LiDAR.  
  Our odometry estimates the robot pose with respect to a base frame, by default assumed to be the LiDAR frame unless you override this with ``--base_frame``.  
  The TF tree is queried for the necessary transformations.

If there is no TF tree, you must supply extrinsics for IMU-to-base and LiDAR-to-base. Set one transformation to identity if you want that frame to be the reference, but both must be given.

Message header frame IDs must match the TF tree frame names. If they do not, override with ``--lidar_frame`` or ``--imu_frame``.

.. code-block:: bash
   # The config should provide extrinsics if they can't be inferred
   rko_lio -v -c config.yaml --imu imu_topic --lidar lidar_topic /path/to/rosbag_folder

Raw Dataloader
--------------

When using the raw dataloader, arrange your dataset directory as follows:

.. code-block:: none

   dataset_root/
   ├── transforms.yaml                 # required: contains 4x4 matrices
   ├── imu.csv / imu.txt               # must match required columns
   └── lidar/                          # folder of point clouds
       ├── 1662622237000000000.ply
       ├── 1662622238000000000.ply
       └── ...

- ``transforms.yaml``: defines two keys (``T_imu_to_base``, ``T_lidar_to_base``), each a 4×4 matrix. See :ref:`Extrinsics and conventions <data-extrinsics-convention>`.
- IMU file: Only one file (CSV or TXT) is allowed. Required columns: ``timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z``. Extra columns are allowed. ``timestamp`` in nanoseconds, others in SI units.
- ``lidar/``: contains scans as PLY files. Each filename is a timestamp (ns) for the scan.  
  Each PLY file must have a time field (accepted names: ``time``, ``timestamp``, ``timestamps``, or ``t``) in **seconds**.

HeLiPR Dataloader
-----------------

This loader is deprecated and will be removed in a future release. If you need it supported, please open an issue.
