# The sensor data

RKO LIO is a LiDAR-inertial odometry system.
Unsurprisingly, we need both IMU and LiDAR data.
It is assumed that both sensors are time synchronized, i.e. the data timestamps refer to the same time clock and are consistent across both sensors.
Some delay in the arrival of the data itself is fine.

## IMU

We only need the measurement time, the accelerometer reading, and the gyroscope reading (from a 6-axis IMU).
Acceleration values should be in m/s^2.
If your accelerometer outputs readings in g's, be sure to convert them to m/s^2 before using the odometry.
Angular velocity values should be in rad/s; if your device uses other units, convert accordingly.

## LiDAR

If your platform experiences rapid or aggressive motions, I strongly recommend enabling deskewing (motion compensation) for good odometry.
Deskewing the LiDAR scan will account for the motion of the platform during the scan acquisition time.
This requires per-point timestamp information.
If these are relative time measurements, we also need either the scan recording start or end time.
Further specifics depend on the specific format of data or the dataloader, but for ROS this translates to requiring the timestamp in the message header and a time field in the point format for a PointCloud2 message.
I attempt to automatically handle the different ways timestamps may be encoded, so most sensor drivers should work out of the box.
Check [here](../cpp/rko_lio/core/process_timestamps.cpp) if you're curious, or if you'd like to contribute a solution for a sensor I don't handle.

## Extrinsic

Finally, we need the extrinsic calibration between the IMU and LiDAR. The convention for specifying extrinsics is covered [here](/README.md#a-note-on-transformations).
