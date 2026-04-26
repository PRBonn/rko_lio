Python
======

.. raw:: html

  <div>
  <p>
    Python Bindings:
  <a href="https://github.com/PRBonn/rko_lio/actions/workflows/python_bindings.yaml"><img src="https://img.shields.io/github/actions/workflow/status/PRBonn/rko_lio/python_bindings.yaml?branch=master&label=Python%20Bindings" alt="Python Bindings" /></a>
  </p>
  </div>

The Python interface is a convenient way to run the odometry on pre-recorded data.

.. contents::
   :local:
   :depth: 2

Setup
-----

Install from PyPI
^^^^^^^^^^^^^^^^^

.. code-block:: bash

   pip install rko_lio

Wheels are published for Linux, macOS, and Windows (`PyPI page <https://pypi.org/project/rko-lio>`_).

To use any of the dataloaders or visualization, install the matching extras.
You'll be prompted at runtime if a required package is missing.
For example, to use the rosbag dataloader and visualize the results:

.. code-block:: bash

   pip install rko_lio rosbags rerun-sdk

Or install everything at once:

.. code-block:: bash

   pip install "rko_lio[all]"

Build from source
^^^^^^^^^^^^^^^^^

Clone the repository and:

.. code-block:: bash

   pip install .

For all optional dependencies:

.. code-block:: bash

   pip install ".[all]"

Or use the convenience recipes in the `Makefile <Makefile>`_:

.. code-block:: bash

   make install    # installs all optional deps
   make editable   # installs an editable version with all deps

The Python build uses ``scikit-build-core``.
You only need Python >= 3.10 and ``pip`` (or another frontend).
Core C++ dependencies (Eigen, Sophus, TBB, nlohmann_json) are fetched automatically; Bonxai is always fetched.
If you want to build against system libraries instead, set ``RKO_LIO_FETCH_CONTENT_DEPS=OFF`` via the scikit-build CMake args -- the default for the Python build is ``ON``.

Usage
-----

For all CLI flags:

.. code-block:: bash

   rko_lio --help

The most common invocation is just:

.. code-block:: bash

   rko_lio /path/to/data

Add ``-v`` to enable visualization (uses `rerun <https://rerun.io>`_; install the ``rerun-sdk`` extra).

There are three dataloaders: ``rosbag`` (ROS1 or ROS2), ``raw``, and ``HeLiPR`` (deprecated). The system tries to detect the right one from the data path; choose explicitly with ``-d``.

A config file is passed with ``--config`` / ``-c``.
Dump a default config to inspect or edit:

.. code-block:: bash

   rko_lio --dump_config

Extrinsics are specified in the config under these keys:

.. code-block:: yaml

   extrinsic_imu2base_quat_xyzw_xyz:   [0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0]
   extrinsic_lidar2base_quat_xyzw_xyz: [0.0, 0.0, 0.0, 1.0,  0.1, 0.0, 0.05]

Both keys are required, **unless** the dataloader can supply the extrinsics itself (e.g. the rosbag dataloader can pull them from a static TF tree in the bag).
The format is ``[qx, qy, qz, qw, x, y, z]``, with the convention described under :ref:`data-extrinsics-convention`.

.. warning::
  If the dataloader provides extrinsics but you specify them in the config, the config values take priority.

If your platform starts at rest, set ``initialization_phase: true`` in the LIO config.
The system uses the IMU between the first two scans to initialise roll / pitch and IMU biases, which makes a noticeable difference -- especially when starting on an inclined surface.

For the full list of LIO and pipeline parameters, see :doc:`config`.
For data assumptions (units, conventions, timestamps), see :doc:`data`.

Dataloaders
-----------

.. automodule:: rko_lio.dataloaders.rosbag
   :no-index:

.. automodule:: rko_lio.dataloaders.raw
   :no-index:

.. automodule:: rko_lio.dataloaders.helipr
   :no-index:

Python API
----------

The full module-level API reference is auto-generated: see :doc:`../python/modules`.

.. toctree::
   :hidden:

   ../python/modules
