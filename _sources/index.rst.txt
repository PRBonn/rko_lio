RKO-LIO - LiDAR-Inertial Odometry
=================================

.. raw:: html

  <div>
    <p>
      <a href="https://arxiv.org/abs/2509.06593"><img src="https://img.shields.io/badge/arXiv-2509.06593-b31b1b.svg" alt="arXiv badge" /></a>
      <a href="/LICENSE"><img src="https://img.shields.io/github/license/PRBonn/rko_lio" alt="GitHub License" /></a>
      <a href="/"><img src="https://img.shields.io/github/last-commit/PRBonn/rko_lio" alt="GitHub Last Commit" /></a>
      <a href="https://pypi.org/project/rko-lio/"><img src="https://img.shields.io/pypi/v/rko_lio?color=blue" alt="PyPI version" /></a>
    </p>
  </div>

.. figure:: ./_static/example_multiple_platforms_shadow.png
   :alt: Visualization of odometry system running on data from four different platforms in four different environments
   :align: center
   :figwidth: 100%

   *Four different platforms, four different environments, one odometry system*


RKO-LIO is a LiDAR-inertial odometry system that is by design simple to deploy on different sensor configurations and robotic platforms with as minimal a change in configuration as necessary.

There is no restriction on which LiDAR you can use, and you can do so without changing any config (I've tested Velodyne, Ouster, Hesai, Livox, Robosense, Aeva sensors).
For using an IMU, I require only the accelerometer and gyroscope readings, the bare minimum.
You don't need to look up manufacturer spec sheets to provide noise specifications, etc.

All you need to provide is the extrinsic transformation between the IMU and LiDAR and you can start using the system for your LiDAR-inertial odometry needs!

.. note::
   This documentation is still under construction. If you see something you can improve, I'd greatly appreciate any help. Please make an issue or a PR on `GitHub <https://github.com/PRBonn/rko_lio>`__!

You can use RKO-LIO in two ways:

1. **Online**, on a robot, via the :doc:`ROS package <pages/ros>`. This is the recommended path. The ROS wrapper handles incoming data streams more efficiently than the Python pipeline and also has an offline mode that drains a rosbag at full speed.
2. **Offline**, on pre-recorded data, via the :doc:`Python package <pages/python>`. Convenient for quick experiments and analysis.

For a short demo, see this video:

.. figure:: ./_static/odometry_video_thumbnail.png
   :target: https://www.youtube.com/watch?v=NNpzXdf9XmU


Citation
^^^^^^^^

If you use this work, please leave a star ⭐ on our `GitHub repository <https://github.com/PRBonn/rko_lio>`_ and consider citing:

.. code-block:: bibtex

  @article{malladi2025arxiv,
    author      = {M.V.R. Malladi and T. Guadagnino and L. Lobefaro and C. Stachniss},
    title       = {A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling},
    journal     = {arXiv preprint},
    year        = {2025},
    volume      = {arXiv:2509.06593},
    url         = {https://arxiv.org/pdf/2509.06593},
  }

The :doc:`paper supplementary material <suppl>` page collects extra figures and explanations pertaining to the RAL paper.

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   Quickstart <pages/quickstart>
   Data <pages/data>
   ROS <pages/ros>
   Python <pages/python>
   Configuring rko_lio <pages/config>
   Changelog <pages/changelog_literal>
   License <__LICENSE>
