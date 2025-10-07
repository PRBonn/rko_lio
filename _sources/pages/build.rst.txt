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
