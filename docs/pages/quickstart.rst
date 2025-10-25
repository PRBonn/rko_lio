The TL;DR
=========

Python
------

.. code-block:: bash

    pip install rko_lio
    rko_lio data_dir

For example, if you’re running on a ROS bag, place all split parts in one folder (with ``metadata.yaml`` for ROS2) and pass that folder as ``data_dir``.

By default, ``rko_lio`` tries to infer topics and TF relationships automatically, but you can override these using flags like ``--lidar``, ``--imu``, or ``--base_frame``.
If your bag doesn’t contain TF information, you’ll need to specify the IMU–LiDAR extrinsics via a config YAML (``rko_lio --dump_config`` will write a default config).

See :doc:`Python Usage <python/usage>` for detailed examples and advanced options.

ROS
---

Supported distros: Humble, Jazzy, Kilted, Rolling

.. code-block:: bash

    sudo apt install ros-${ROS_DISTRO}-rko-lio
    ros2 launch rko_lio odometry.launch.py

Please see :doc:`build from source <ros/setup>` if you want to do that instead.

The launch file supports both online and offline modes via the ``mode`` argument and expects at least ``lidar_topic``, ``imu_topic``, and ``base_frame``.

You can override configuration parameters through CLI or a separate YAML file (e.g., ``config_file:=/path/to/config``).

For visualization, add ``rviz:=true``.

To explore all launch parameters:

.. code-block:: bash

    ros2 launch rko_lio odometry.launch.py -s

See :doc:`ROS Usage <ros/usage>` for an extended description.
