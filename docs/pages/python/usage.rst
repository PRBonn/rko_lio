Python - Usage
==============

You can pass a number of options to ``rko_lio``.
For all possible CLI flags, run:

.. code-block:: bash

   rko_lio --help

The ``-v`` flag enables visualization.

There are three dataloaders available: ``rosbag`` (ROS1 or ROS2), ``raw``, and ``HeLiPR`` (deprecated).
For most usages, the system will automatically detect which dataloader to use from your data path, but you can choose explicitly with ``-d``.

A config file can be passed with ``--config`` or ``-c``. Dump a config file with all default options using

.. code-block:: bash

   rko_lio --dump_config

Extrinsic transformations must be specified with the following keys in the config:

- ``extrinsic_imu2base_quat_xyzw_xyz``
- ``extrinsic_lidar2base_quat_xyzw_xyz``

These are **required parameters**. But if the dataloader can provide the extrinsics automatically, then they are **not required**.

.. warning::
  If your dataloader provides extrinsics, but you specify them in a config, the config values *will* take priority**.
