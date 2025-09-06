# Configuration Parameters

The odometry has a set of runtime parameters that are common across both the Python bindings and the ROS interface.

The defaults are sane, and I've used the defaults with success across a number of platforms, datasets, etc.

But your specific application might still benefit from or need some tuning, so I've given some brief description below.

The physical units are always SI units.

- **deskew** (`bool`, default `True`)
    Whether to apply scan deskewing before registration.
    This compensates for the motion that happens during LiDAR scan collection, since your platform is probably moving while the LiDAR is collecting data.
    Unless you have very good reason, always keep this enabled.
    This **_requires_** per-point timestamps in the LiDAR scan, i.e., the scan needs to have `xyzt` per-point where `t` is time.
    If you cannot provide this, then deskewing has to be disabled.

- **double_downsample** (`bool`, default `True`)
    Useful for dense LiDARs. Disabling this for sparse sensors, like a VLP-16 compared to an Ouster-128, can potentially improve results.
    Indoor scenes can also see an improvement by disabling this.

- **voxel_size** (`float`, default `1.0`)
    The voxel resolution of the internal local map (in meters).
    Smaller values create finer maps at higher memory/computation cost.
    Reducing this for sparse sensors or indoor scenes can help improve results.

- **max_range** (`float`, default `100.0`)
    Maximum usable LiDAR range in meters.
    Points beyond this cutoff are ignored.
    Reducing this is an easy way to reduce compute requirements, since you typically won't need information from 100m away for odometry.
    Nevertheless, this is the default.

- **min_range** (`float`, default `1.0`)
    Minimum LiDAR range in meters.
    Points closer than this are discarded.
    Useful if your platform shows up in the scan due to occlusions.

- **max_points_per_voxel** (`int`, default `20`)
    Maximum number of points stored per voxel in the VDB map.
    Affects both memory and ICP data association.
    In case you need more runtime performance, you can reduce this.
    Odometry performance will be a bit affected, but how much depends on the environment.

- **max_correspondance_distance** (`float`, default `0.5`)
    Maximum distance threshold (meters) for ICP data associations.

- **max_iterations** (`int`, default `100`)
    Limit on the number of iterations for ICP.
    You can limit this a bit more if runtime is an issue.
    Typically the convergence criterion is satisfied much earlier anyways.

- **convergence_criterion** (`float`, default `1e-5`)
    Termination criterion for optimization.
    Lower (stricter) values will requires more ICP iterations.

- **max_num_threads** (`int`, default `0`)
    Only used to parallelize data association for ICP.
    `0` means autodetect based on hardware.
    In case, compute resources are a constraint, limit this to a few threads and, in order, `max_points_per_voxel`, `voxel_size`, `max_range`, `max_iterations` are the parameters you probably care about.

- **initialization_phase** (`bool`, default `True`)
    Initializes the system orientation (roll and pitch) plus IMU biases using the IMU measurements between the first two LiDAR scans the odometry receives.
    If enabled, the second frame is assumed to be coincident with the first. I.e., the assumption is that the system is at rest for that duration and the system is oriented to align with gravity.
    This helps if you start from an inclined surface for example.
    Usually you can leave this enabled. Unless for some reason you need to start the odometry while the system is in motion, then disable this.

- **max_expected_jerk** (`float`, default `3.0`)
    This value is used in a Kalman filter to estimate the true body acceleration. It should reflect the motion you expect from the platform you will deploy the odometry on. A good range is [1-3] m/s^3, but it should be fine to leave it at 3 m/s^3 as that is a good setting for most platforms.

- **min_beta** (`float`, default `200.0`)
    The minimum weight applied to an orientation regularization cost during scan alignment.
    Essentially we use the accelerometer readings as an additional observation on the roll and pitch of the system (we need to estimate the true body acceleration using the Kalman filter mentioned above).
    This parameter influences how much importance this additional observation plays in the optimzation, as we cannot have a perfect observation of the true body acceleration (the estimate is affected by gravity and the odometry itself).
    The default should be fine for most cases.
    You can set it -1 to disable this additional cost.
