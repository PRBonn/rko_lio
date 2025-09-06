# Build Specifics

The package is split into three parts, a core C++ library in `cpp`, which implements all the LiDAR-inertial odometry logic, and the python bindings in `python` and the ROS interface in `ros`, which essentially handle reading data.

The most important build requirement is CMake>=3.28, due to how we handle core dependencies.
This is the default CMake version on Ubuntu 24.04 (also the ROS Jazzy target Ubuntu platform).
If you are on older systems, your options to satisfy the version are either `pip install cmake` or to build CMake from source.

Additionally, in the different build options we provide ([cpp](Makefile#L7), [python](python/pyproject.toml#L50), [ros](ros/colcon.pkg#L2)), we default to using `ninja` as a generator.
You can switch this out if required.

In summary, `CMake>=3.28` and `ninja` (optional) are the two build requirements.
The python build also handles the CMake and ninja requirements.

## The core library

The core library dependencies are:
- [Bonxai](https://github.com/facontidavide/Bonxai): for the VDB map
- [Intel oneTBB](https://github.com/uxlfoundation/oneTBB): used for optionally parallelizing the data association in ICP
- [nlohmann_json](https://github.com/nlohmann/json): for dumping some logs to disk
- [Sophus](https://github.com/strasdat/Sophus): for Lie Group math
- Eigen

There's two ways to build the core library.
Either you provide **all** our dependencies yourself as system packages ([the default](cpp/CMakeLists.txt#L28)), or we can vendor them using CMake's `FetchContent`.
We don't support partial fetches, i.e., all or nothing.
Except, we **currently always fetch Bonxai** (see [here](cmake/dependencies.cmake#L30)) as the upstream itself doesn't provide a way to install Bonxai as a system package.

Our dependency management is opt-in and you can pass `-DDRKO_LIO_FETCH_CONTENT_DEPS=ON` to the CMake configure step to enable it.

## The python bindings

The python build uses `scikit-build-core` and all you need to provide is a python version >=3.9 and `pip` (or other build frontend).

Then you can do `cd python && pip install .` and you don't even need to provide CMake or ninja or anything else.
By default we fetch the core library dependencies, which you can change [here](python/pyproject.toml#L66).

If you want an editable install, then check [here](python/Makefile#L7).

## The ROS interface

We currently support (and have tested on) ROS Jazzy.

If you have a full ROS environment already set up, nothing much is different from the core library build section except that we use colcon now (and you still need CMake>=3.28).

There are no special ROS dependencies, the core library dependencies still apply, but you can check [here](https://github.com/mehermvr/rko_lio/blob/d626d0a6b8fa3883ef0fad0d604562d0062ef3f2/ros/package.xml#L11) and use `rosdep` if required for some reason.

I provide some default CMake arguments for the colcon build [here](ros/colcon.pkg#L3), which you can override by passing command line flags to colcon.
