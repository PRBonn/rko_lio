The TL;DR
=========

Python
------

.. code-block:: bash

    pip install "rko_lio[all]"
    rko_lio -v /path/to/data

For example, if you're running on a ROS bag, place all split parts in one folder (with ``metadata.yaml`` for ROS2) and pass that folder as the data path.
``-v`` opens the rerun visualizer.

By default, ``rko_lio`` tries to infer topics and TF relationships automatically; override with flags like ``--lidar``, ``--imu``, or ``--base_frame``.
If your bag doesn't contain TF, supply IMU--LiDAR extrinsics via a YAML config (``rko_lio --dump_config`` writes a default one to start from).

See :doc:`Python <python>` for detailed examples and the full CLI surface.

ROS
---

Supported distros: Humble, Jazzy, Kilted, Rolling.

.. code-block:: bash

    sudo apt install ros-${ROS_DISTRO}-rko-lio
    ros2 launch rko_lio odometry.launch.py \
        lidar_topic:=/your/lidar imu_topic:=/your/imu base_frame:=base

If you want to build from source instead, see :doc:`ROS <ros>`.

The launch file supports both online and offline modes via the ``mode`` argument and requires at least ``lidar_topic``, ``imu_topic``, and ``base_frame``.
Override individual parameters on the CLI or load them from a YAML file (``config_file:=/path/to/config``).
Add ``rviz:=true`` for visualization.

To explore all launch parameters:

.. code-block:: bash

    ros2 launch rko_lio odometry.launch.py -s

See :doc:`ROS <ros>` for an extended description.
