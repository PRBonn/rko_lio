Core dependencies
=================

RKO-LIO is split into three parts:

- a core C++ library in ``core``, which implements all the LiDAR-inertial odometry logic,
- the Python bindings in ``python``, and
- the ROS interface in ``ros``.

With how the package is structured, the python and ros interfaces essentially handle the data reading problem, and the core library handles the odometry part.

I highly recommend using ``cmake>=3.28``, due to how I handle core library dependencies.
This is the default CMake version on Ubuntu 24.04 (also the ROS Jazzy/Kilted target Ubuntu platform).

Building with ``cmake>=3.22`` is still supported, which is the default version on Ubuntu 22.04 (if you're using Humble), but it is less tested.
In case you encounter build problems, please open an issue and I'll look into it.

Using ``ninja`` as a generator is also recommended.
On ubuntu, you can install it with

.. code-block:: bash

   sudo apt install ninja-build

The python build will automatically use both ``cmake>=3.28`` and ``ninja``.

The core library
----------------

The core library dependencies are:

- `Bonxai <https://github.com/facontidavide/Bonxai>`__: for the VDB map
- `Intel oneTBB <https://github.com/uxlfoundation/oneTBB>`__: optionally parallelizes data association in ICP
- `nlohmann_json <https://github.com/nlohmann/json>`__: for dumping some logs to disk
- `Sophus <https://github.com/strasdat/Sophus>`__: for Lie Group math
- `Eigen <https://eigen.tuxfamily.org>`__: Self-explanatory

There are two ways to build the core library:

1. Provide **all** dependencies yourself as system packages (default, see ``cpp/CMakeLists.txt`` line 28, also see exception below).
2. Vendor dependencies using CMake's ``FetchContent``.

Using FetchContent for partial fetches is not supported: it's all or nothing.

.. note::
   Bonxai is **always fetched** (see ``cmake/dependencies.cmake`` line 30) because upstream does not provide a system package.


In general, I recommend using option 2, as I specify the versions for all dependencies and you'll get a more consistent experience across systems.
The python interface by default always fetches the deps.
However, if you're using the ros binaries from the ros' repositories (``sudo apt install ros-${ROS_DISTRO}-rko-lio``), then you'll be getting the versions of libraries that come packaged with your system.

FetchContent dependency management is opt-in; pass ``-DRKO_LIO_FETCH_CONTENT_DEPS=ON`` to the CMake configure step to enable it.

Upgrading CMake to version 3.28
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case:

- you're on an older system
- your default system package manager doesn't provide CMake v3.28
- and you'd like to upgrade

Then you can use the following commands to build CMake from source and install it.
If you are doing this anyways, I'd recommend also bumping the CMake version to its latest version as they are usually backwards compatible.

.. code-block:: bash

   export CMAKE_VERSION="3.28.6"
   cd /tmp
   wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz
   tar -zxf cmake-${CMAKE_VERSION}.tar.gz
   rm cmake-${CMAKE_VERSION}.tar.gz
   cd cmake-${CMAKE_VERSION}
   ./bootstrap --system-curl --prefix=/usr/local
   make -j$(nproc)
   sudo make install
   cd /tmp
   rm -rf cmake-${CMAKE_VERSION}
   cmake --version
