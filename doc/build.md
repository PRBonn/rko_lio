# Build Specifics

The package is split into three parts, a core C++ library in `cpp`, which implements all the LiDAR-inertial odometry logic, and the python bindings in `python` and the ROS interface in `ros`, which essentially handle reading data.

I highly recommend you use `cmake>=3.28`, due to how I handle core library dependencies.
This is the default CMake version on Ubuntu 24.04 (also the ROS Jazzy/Kilted target Ubuntu platform).
We still support building with `3.22`, which is the default version on Ubuntu 22.04 (if you're using humble), but this is less tested.
In case you encounter some build problems, please report it, and I'll look into it.

I also recommend using `ninja` as a generator.
On ubuntu, you can install it with

```bash
sudo apt install ninja-build
```

The python build will automatically use both `cmake>=3.28` and `ninja`.

## The core library

The core library dependencies are:
- [Bonxai](https://github.com/facontidavide/Bonxai): for the VDB map
- [Intel oneTBB](https://github.com/uxlfoundation/oneTBB): used for optionally parallelizing the data association in ICP
- [nlohmann_json](https://github.com/nlohmann/json): for dumping some logs to disk
- [Sophus](https://github.com/strasdat/Sophus): for Lie Group math
- Eigen

There's two ways to build the core library.
Either you provide **_all_** our dependencies yourself as system packages ([the default](cpp/CMakeLists.txt#L28)), or we can vendor them using CMake's `FetchContent`.
We don't support partial fetches, i.e., it's all or nothing.
Except Bonxai.
We **currently always fetch Bonxai** (see [here](cmake/dependencies.cmake#L30)) as the upstream itself doesn't provide a way to install it as a system package.

Our dependency management is opt-in and you can pass `-DRKO_LIO_FETCH_CONTENT_DEPS=ON` to the CMake configure step to enable it.

## The python bindings

The python build uses `scikit-build-core` and all you need to provide is a python version >=3.10 and `pip` (or other build frontend).

Then you can do `cd python && pip install .` and you don't even need to provide CMake or ninja or anything else.
By default we fetch the core library dependencies, which you can change [here](python/pyproject.toml#L66).

If you want an editable install, then check [here](python/Makefile#L7).

## The ROS interface

We currently support ROS Jazzy and Kilted.

If you have a full ROS environment already set up, nothing much is different from the core library build section except that we use colcon now (and you still need CMake>=3.28).

There are no special ROS dependencies, the core library dependencies still apply, but you can check [here](https://github.com/mehermvr/rko_lio/blob/d626d0a6b8fa3883ef0fad0d604562d0062ef3f2/ros/package.xml#L11) and use `rosdep` if required for some reason.

I provide some default CMake arguments for the colcon build [here](ros/colcon.pkg#L3), which you can override by passing command line flags to colcon.

## Upgrading CMake to version 3.28

In case you're on older systems, and your default system package manager doesn't provide CMake v3.28, you can use the following commands to build CMake from source and install it.
If you are doing this anyways, I'd recommend also bumping the CMake version to latest.

```bash
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
```
