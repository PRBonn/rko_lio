Python - Setup
==============

Install from PyPI
-----------------

Simply run:

.. code-block:: bash

   pip install rko_lio

I provide PyPI wheels for Linux, macOS, and Windows (`PyPI page <https://pypi.org/project/rko-lio>`_).

To be able to use ``rko_lio`` with any of the dataloaders or to enable visualization, you'll need to install additional dependencies.
You'll be prompted for specific packages as they become required during runtime.

For example, to use the rosbag dataloader and visualize the results, run:

.. code-block:: bash

   pip install rko_lio rosbags rerun-sdk

If you want to install everything at once, use:

.. code-block:: bash

   pip install "rko_lio[all]"

Build from Source
-----------------

Clone the repository and then run:

.. code-block:: bash

   pip install .

To have all optional dependencies installed, run:

.. code-block:: bash

   pip install ".[all]"

Or use the convenience recipes provided in the `Makefile <Makefile>`_:

.. code-block:: bash

   make install    # installs all optional deps
   make editable   # installs an editable version with all deps

Advanced Details
----------------

The Python build uses ``scikit-build-core``.
You only need Python ≥ 3.10 and ``pip`` (or another frontend).
All core dependencies are fetched automatically unless you modify the option in ``pyproject.toml`` (see also :doc:`Core Dependencies <../uncategorised/core>`).

For an editable install recipe, see ``Makefile`` which will setup other build dependencies.
