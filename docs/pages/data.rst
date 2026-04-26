The sensor data
===============

RKO-LIO is a LiDAR-inertial odometry system, so unsurprisingly we need both IMU and LiDAR data.
This page describes what each stream needs to look like, what time synchronisation we expect, and the convention we use for the IMU<->LiDAR extrinsic.

Both sensors must be time-synchronised: the timestamps on the data refer to the same clock and are consistent across both streams.
A bit of latency between when a measurement is sampled and when it arrives in the system is fine -- ROS message buffering / Python sequencing handles that -- but the timestamps themselves must be aligned.

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

What we need: timestamp, accelerometer reading (3-axis), gyroscope reading (3-axis). A 6-axis IMU is enough; we don't use orientation, magnetometer, or noise / bias spec sheets.

- **Acceleration** in m/s². If your accelerometer reports g's, convert before feeding it in.
- **Angular velocity** in rad/s. Convert if your device uses other units.

.. warning::

  We follow the typical ROS convention for the IMU (see `ROS REP 145 <https://www.ros.org/reps/rep-0145.html>`__).
  At rest in a neutral orientation -- IMU and world frames aligned -- the IMU *z* axis points upwards and the accelerometer measures **+g m/s² along +z**.

LiDAR
-----

What we need: a point cloud per scan, plus per-point timestamps (seconds) for every point in the scan.
Per-point timestamps are what lets us deskew the scan against the platform motion during the scan acquisition window.

If your platform sees rapid or aggressive motion, deskewing matters a lot -- enable it (``deskew=True``, the default).
If you cannot supply per-point timestamps, you must disable deskewing and take the quality hit.

For ROS, this translates to:

- A timestamp in the ``PointCloud2`` message header (the standard sensor driver pattern).
- A time field per point inside the cloud. Common field names are accepted (``time``, ``timestamp``, ``timestamps``, ``t``).

I try to handle the different ways drivers encode per-point timestamps automatically -- absolute (already aligned with wall-clock time) vs. relative (offsets from the message header time), seconds vs. nanoseconds, and so on.
The heuristic is at `rko_lio/core/process_timestamps.cpp <https://github.com/PRBonn/rko_lio/blob/master/rko_lio/core/process_timestamps.cpp>`_.

If the heuristic can't classify your data, the system throws a ``Runtime Error: TimestampProcessingConfig does not cover this particular case of data``.
You can override the behaviour via the ``timestamps:`` block in your config (Python) or the ``lidar_timestamps.*`` parameters (ROS) -- see :ref:`Configuring -> LiDAR per-point timestamps <config-lidar-timestamps>`.
And if your sensor is different and you think the heuristic should handle it, please open an issue (or a PR).
