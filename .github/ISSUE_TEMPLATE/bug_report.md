---
name: Bug report
about: Please use this if rko lio has an error, otherwise use the blank template
title: "[BUG]"
labels: bug
assignees: ''

---

<!--
If you think your issue doesn't require all the following details, use the blank issue template instead.
Please do note however, if you do fill in all or most of the following details, you make life easier for me to debug the error.
Otherwise, I might have to ask you some of these details again anyways.
-->

**Environment**
- OS?
- Python version (eg., 3.10) or ROS distro (e.g., Jazzy)?


**RKO LIO**
- How did you install/build RKO LIO?
- If you are building from source, which branch/version/commit?
- How are you running RKO LIO (give the full `rko_lio` or `ros2 launch/run` command).
- (ROS only) If you're using the launch file, please copy the Launch configuration that gets printed to console (between `====`) when you run it (no need to include the launch file contents itself).
```text
====
launch configuration console log
====
```
- (Python only) If you're using a config (yaml) file, please include the contents here
```yaml
contents of your config file here
```


**Data**
- (ROS only) Is there a TF tree which specifies extrinsics?
- Are you specifying the LiDAR-IMU extrinsic manually?
- Any other data or sensor specific information that is important to know?

**Steps to reproduce**

Please detail any steps you think are necessary to reproduce the error.

**Any other information**

Please include any other information you want to from here.
