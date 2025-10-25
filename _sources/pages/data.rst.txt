The sensor data
===============

RKO LIO is a LiDAR-inertial odometry system.
Unsurprisingly, we need both IMU and LiDAR data.
It is assumed that both sensors are time synchronized, i.e. the data timestamps refer to the same time clock and are consistent across both sensors.
Some delay in the arrival of the data itself is fine.

.. _data-extrinsics-convention:

Extrinsics & convention
-----------------------

Most importantly, we need the extrinsic calibration between the IMU and LiDAR.

Throughout this package, we refer to transformations using ``transform_<from-frame>_to_<to-frame>`` or ``transform_<from-frame>2<to-frame>``. By this, we mean a transformation that converts a vector expressed in the ``<from-frame>`` coordinate system to the ``<to-frame>`` coordinate system.

Mathematically, this translates to:

.. math::

   \mathbf{v}^{\text{to}} = {}^{\text{to}} \mathbf{T}_{\text{from}}  \mathbf{v}^{\text{from}}

The superscript on the vector indicates the frame in which the vector is expressed, and
:math:`{}^{\text{to}} \mathbf{T}_{\text{from}}` corresponds to ``transform_<from-frame>_to_<to-frame>``.

IMU
---

We only need the measurement time, the accelerometer reading, and the gyroscope reading (from a 6-axis IMU).

Acceleration values should be in m/s².
If your accelerometer outputs readings in g's, be sure to convert them to m/s² before using the odometry.

Angular velocity values should be in rad/s; if your device uses other units, convert accordingly.

LiDAR
-----

If your platform experiences rapid or aggressive motions, I strongly recommend enabling deskewing (motion compensation) for good odometry.
Deskewing the LiDAR scan will account for the motion of the platform during the scan acquisition time.
This requires per-point timestamp information.
If these are relative time measurements, we also need either the scan recording start or end time.
Further specifics depend on the specific format of data or the dataloader, but for ROS this translates to requiring the timestamp in the message header and a time field in the point format for a PointCloud2 message.

I attempt to automatically handle the different ways timestamps may be encoded, so most sensor drivers should work out of the box.
Check `here <../cpp/rko_lio/core/process_timestamps.cpp>`__ if you're curious, or if you'd like to contribute a solution for a sensor I don't handle.
