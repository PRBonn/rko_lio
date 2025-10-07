Building from source
====================

The package is split into three parts:

- a core C++ library in ``cpp``, which implements all the LiDAR-inertial odometry logic,
- the Python bindings in ``python``, and
- the ROS interface in ``ros``.

With how the package is structured, the python and ros interfaces essentially handle the data reading problem, and the core library handles the odometry part.

I highly recommend using ``cmake>=3.28``, due to how I handle core library dependencies.
This is the default CMake version on Ubuntu 24.04 (also the ROS Jazzy/Kilted target Ubuntu platform).

I still support building with ``3.22``, which is the default version on Ubuntu 22.04 (if you're using Humble), but it is less tested.
In case you encounter build problems, please open an issue and I'll look into it.

I also recommend using ``ninja`` as a generator.
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

.. admonition:: Note
   :class: note

   Bonxai is **always fetched** (see ``cmake/dependencies.cmake`` line 30) because upstream does not provide a system package.


In general, I recommend using option 2, as I specify the versions for all dependencies and you'll get a more consistent experience across systems.
The python interface by default always fetches the deps.
However, if you're using the ros binaries from the ros' repositories (``sudo apt install ros-distro-rko-lio``), then you'll be getting the versions of libraries that come packaged with your system.

FetchContent dependency management is opt-in; pass ``-DRKO_LIO_FETCH_CONTENT_DEPS=ON`` to the CMake configure step to enable it.

The Python bindings
-------------------

The Python build uses ``scikit-build-core``.  
You only need Python version >=3.10 and ``pip`` (or another frontend).

Simply run:

.. code-block:: bash

   cd python && pip install .

and you won’t need to provide CMake, Ninja, or anything else.

By default, the core library dependencies are fetched; you can change that in ``python/pyproject.toml`` line 66.

For an editable install, see line 7 of ``python/Makefile``.

The ROS interface
-----------------

I support ROS Humble, Jazzy, Kilted, and Rolling.

If you have a full ROS environment set up, the process is similar to the core library build except we use colcon now.

You can use ``rosdep`` to install all the dependencies. For example, run the following in your ROS workspace directory (where the `src` folder is located):

.. code-block:: bash

   rosdep install --from-paths src --ignore-src -r -y

I provide certain default colcon CMake arguments are provided in ``colcon.pkg`` which you **might not want**.
In this case, please edit the file to disable any flag.

Upgrading CMake to version 3.28
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case you're on older systems, and your default system package manager doesn't provide CMake v3.28, you can use the following commands to build CMake from source and install it.
If you are doing this anyways, I'd recommend also bumping the CMake version to latest.

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
