ROS - Setup
===========

Install Pre-Built Releases
--------------------------

Supported distros: Humble, Jazzy, Kilted, Rolling

.. code-block:: bash

    sudo apt install ros-${ROS_DISTRO}-rko-lio

Build from source
-----------------

The system dependencies are:

- CMake, ROS environment
- Optionally: Eigen, Sophus, nlohmann_json, TBB, Bonxai (see :doc:`../core_dependencies` for more details)

Clone the repository into a colcon workspace’s ``src``.

You can handle dependencies through either:

- rosdep
- FetchContent

For rosdep, run the following in your workspace directory:

.. code-block:: bash

   rosdep install --from-paths src --ignore-src -r -y

For using FetchContent, enable the ``RKO_LIO_FETCH_CONTENT_DEPS`` option when building.

I provide certain default colcon CMake arguments in ``colcon.pkg`` which you **might not want**:

- ``CMAKE_BUILD_TYPE=Release`` -> self explanatory
- ``RKO_LIO_BUILD_ROS=ON`` -> default ``ON``, will only build the ROS wrapper when enabled
- ``CMAKE_EXPORT_COMPILE_COMMANDS=ON`` -> useful for development

These flags will get picked up automatically when you build the package with colcon.
In case you don't want certain options, please edit the file to disable any flag.

From the workspace folder, then:

.. code-block:: bash

   colcon build --packages-select rko_lio  # --symlink-install --event-handlers console_direct+

In case you didn't choose rosdep to setup dependencies, then instead run

.. code-block:: bash

   colcon build --packages-select rko_lio --cmake-args -DRKO_LIO_FETCH_CONTENT_DEPS=ON


Also consider using ``Ninja`` as a generator to speed up builds instead of Make.
If you do use Make, make sure (pun intended) to parallelize the build.
