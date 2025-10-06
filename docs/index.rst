RKO-LIO: LiDAR-inertial odometry
================================

RKO-LIO is a LiDAR-inertial odometry system that is by design simple to deploy on different sensor configurations and robotic platforms with as minimal a change in configuration as necessary.

I have no restriction on which LiDAR you can use, and you can do so without changing any config (I've tested Velodyne, Ouster, Hesai, Livox, Robosense, Aeva sensors).
For using an IMU, I require only the accelerometer and gyroscope readings, the bare minimum.
You don't need to look up manufacturer spec sheets to provide noise specifications, etc.

All you need to provide is the extrinsic transformation between the IMU and LiDAR and you can start using the system for your LiDAR-inertial odometry needs!

.. admonition:: Note
   :class: note

   This documentation is still under construction. If you see something you can improve, I'd greatly appreciate any help. Please make an issue or a PR on `GitHub <https://github.com/PRBonn/rko_lio>`__!

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   Installation <source/install>
   Usage <source/usage>
   Build from source <source/build>
   Configuring rko_lio <source/config>
   C++ API <generated/index>
   Python API <python/modules>

