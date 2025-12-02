^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rko_lio
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2025-12-02)
------------------
* Fix (core): log proper pose for the first frame if init. phase is on (`#89 <https://github.com/PRBonn/rko_lio/issues/89>`_)
* chore: bump FetchContent Eigen to 5.0.1 (`#88 <https://github.com/PRBonn/rko_lio/issues/88>`_)
  Co-authored-by: github-actions[bot] <github-actions[bot]@users.noreply.github.com>
* core: minimize TimestampProcessingConfig (`#87 <https://github.com/PRBonn/rko_lio/issues/87>`_)
  * remove the configs for start and end time thresholds that can be satisfied much simpler by just forcing relative or absolute overrides
* python: remove unnecessary scan viz (`#85 <https://github.com/PRBonn/rko_lio/issues/85>`_)
* remove pytest as optional dep, added toml based config (`#83 <https://github.com/PRBonn/rko_lio/issues/83>`_)
* Rename published_deskewed_cloud to publish_deskewed_scan (`#81 <https://github.com/PRBonn/rko_lio/issues/81>`_)
* Fix: core - proper second frame pose if initialization off (`#77 <https://github.com/PRBonn/rko_lio/issues/77>`_)
  * move preproc to its own file and fix the non intialization behaviour
  * better (but not cross platform-worthy) identity registration testing
* Update accelerometer data expectation in docs. Clarified accelerometer assumptions
* core: Configurable lidar timestamp processing (`#74 <https://github.com/PRBonn/rko_lio/issues/74>`_)
  * thresholds and timestamp processing behaviour is configurable
  * config modifications to the python side
  * ros side launch config modified to have more params
  * docs additions and cleanup
  * fix the py tests since the api changed
* update installation instructions and badge formatting. Updated README.md to improve formatting and clarity of installation instructions and dependencies.
* More CLI options, profiles stats has variance, dumping deskewed scans (`#73 <https://github.com/PRBonn/rko_lio/issues/73>`_)
  * add the modifications needed to dump the plys when running
  * add variance to profile stats
  * python: pybind interval stats, and imu logging in rerun
  * more cli options, viz improvements a bit
  * update the cli for dumping deskewed
  * update the rbl
* Update README.md. Add ros source build details
* Folder reorganisation and docs update (`#69 <https://github.com/PRBonn/rko_lio/issues/69>`_)
  * flatten the directories. massive renaming across the board
  * docs rework
  * docs updates
  * rename the config file so its explicit
  * fix workflows
* Core/move logging to wrappers (`#68 <https://github.com/PRBonn/rko_lio/issues/68>`_)
  * remove logging from core. improve docs
  * python side now does its own config and traj dumping
  * nlohmann json is no longer a core requirement. ros only
  * result dumping on ros side now
  * make result logging on ros optional. and add launch config for that
  * use a shutdown callback for ros to dump results
* Contributors: Anthony Bisulco, Meher Malladi, dependabot[bot]

0.1.7 (2025-10-21)
------------------
* python: Update rosbags requirement from to allow 0.11 (`#63 <https://github.com/PRBonn/rko_lio/issues/63>`_)
  * Update rosbags requirement from <0.11,>=0.10 to >=0.10,<0.12 in /python
  * Changes to rosbag reader to support older versions and 0.11
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
  Co-authored-by: Meher Malladi <rm.meher97@gmail.com>
* actions: Bump pypa/cibuildwheel from 3.2.0 to 3.2.1 (`#61 <https://github.com/PRBonn/rko_lio/issues/61>`_)
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* python: Update typer requirement from <0.20,>=0.19 to >=0.19,<0.21(`#62 <https://github.com/PRBonn/rko_lio/issues/62>`_)
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* python: Update rerun-sdk version (`#64 <https://github.com/PRBonn/rko_lio/issues/64>`_)
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* python: pin versions for compatibility (`#60 <https://github.com/PRBonn/rko_lio/issues/60>`_)
  * add python to dependabot checks
  * specify upper bounds for pip dependencies
  * add optional dataloader tests, and add a workflow to install and test
  * improve the testing a bit
  * remove the ament cmake find outside of a ros build, as that causes issues on cursed ros setups
* python: data loader improvements and raw loader overhaul (`#56 <https://github.com/PRBonn/rko_lio/issues/56>`_)
* docs: overhaul and add a github pages workflow (`#55 <https://github.com/PRBonn/rko_lio/issues/55>`_)
  essentially a massive overhaul to everything documentation related. we now use rosdocs2, sphinx, doxygen to build docs. and gh-pages-action to deploy to github-pages automatically
* Contributors: Meher Malladi, dependabot[bot]

0.1.6 (2025-10-07)
------------------
* make fPIC a property target ON instead of a global build flag
* drop cmake min version to 3.22.0
* clean up bonxai finding and mocking a bit
* force include bonxai code when not fetching
* Contributors: Meher Malladi

0.1.5 (2025-10-04)
------------------
* Drop cmake min version to 3.22.1 to support ros humble builds with default toolchains  (`#52 <https://github.com/PRBonn/rko_lio/issues/52>`_)
  * mock a find package file if cmake below version 3.24
  * different sophus version compat for older cmake
  * add compat target sources for cmake < 3.23
  * missing include fix
  * cmake policy to suppress warning on the older cmake
  * fix the bonxai fetch problem for ubuntu 22.04 isolated builds
* Contributors: Meher Malladi

0.1.4 (2025-10-04)
------------------
* Drop ros cmake min version to 3.26.5.  (`#50 <https://github.com/PRBonn/rko_lio/issues/50>`_)
  * Conditional fetchcontent flags to include exclude_from_all only if version > 3.28. Should fix rhel 9 builds for ros
* Bump FetchContent dependencies (`#49 <https://github.com/PRBonn/rko_lio/issues/49>`_)
  * bump Eigen FetchContent to 5.0
  ---------
  Co-authored-by: github-actions[bot] <github-actions[bot]@users.noreply.github.com>
  Co-authored-by: Meher Malladi <rm.meher97@gmail.com>
* Contributors: Meher Malladi, github-actions[bot]

0.1.3 (2025-10-02)
------------------
* temp fix: ros builds when FETCHCONTENT_FULLY_DISABLED is ON, aka the ros build farm (`#45 <https://github.com/PRBonn/rko_lio/issues/45>`_)
  * clean up ament target dependencies deprecation
  * update bonxai version
  * add bonxai core source to dependencies/bonxai with a check to include only on specific conditions
* Change package layout to make ros package release easier (`#44 <https://github.com/PRBonn/rko_lio/issues/44>`_)
  * move ros/cmake and package xml out. merge core cmake into root cmake
  * build ros by default, but disable it for the python build
  * makefile recipe disables ros build by default for cpp only build
  * clean up ros workflows a bit
* Bump pypa/cibuildwheel from 3.1.4 to 3.2.0 (`#42 <https://github.com/PRBonn/rko_lio/issues/42>`_)
  Bumps [pypa/cibuildwheel](https://github.com/pypa/cibuildwheel) from 3.1.4 to 3.2.0.
  - [Release notes](https://github.com/pypa/cibuildwheel/releases)
  - [Changelog](https://github.com/pypa/cibuildwheel/blob/main/docs/changelog.md)
  - [Commits](https://github.com/pypa/cibuildwheel/compare/v3.1.4...v3.2.0)
  ---
  updated-dependencies:
  - dependency-name: pypa/cibuildwheel
  dependency-version: 3.2.0
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* Contributors: Meher Malladi, dependabot[bot]

0.1.2 (2025-09-26)
------------------
* Drop irregular lidar frames (`#35 <https://github.com/PRBonn/rko_lio/issues/35>`_)
  * if incoming lidar scan has a stamp with too big of a delta to the previous, we treat it as an error
* fix component registration so online node component shows up in ros2 component types (`#34 <https://github.com/PRBonn/rko_lio/issues/34>`_)
* Contributors: Meher Malladi

0.1.1 (2025-09-17)
------------------
* Fix logic error in offline node (ros) and interval imu stats compute (core) (`#29 <https://github.com/PRBonn/rko_lio/issues/29>`_)
  * cleaner error messages and log parsed imu and lidar frames for ros
  * patch rviz config to change the global frame if a different odom frame is used
  * revert offline node bag and buffer completion handling logic
  * cannot compute accel mag variance if imu count is 1. modify the check preventing divide by 0 errors
  * remove python/config/default.yaml since --dump_config does the same job
  * bump version to v0.1.1 and update python readme slightly
* fix: ros launch - correctly overload config file params with cli params (`#28 <https://github.com/PRBonn/rko_lio/issues/28>`_)
  * fix: ros launch - correctly overload config file params with cli params
  * clarify that the default.yaml config file is not automatically picked up by the ros launch file
  * add a check to make sure 0 or both extrinsics are specified
* Update docs to specify more details on data format (`#25 <https://github.com/PRBonn/rko_lio/issues/25>`_)
  * print the extrinsics to console after guessing them, so that its easier for the user to respecify in a config if required
  * add a data.md that specifies the minimal requirements on data we have.
  * add a readme section for all the configurations rko lio has been tested on
* Update defaults to provide an easier entry into using the system (`#24 <https://github.com/PRBonn/rko_lio/issues/24>`_)
  * set initialization_phase to false by default, as a false start can break odometry. the user can opt-in to initializing if they know their system starts from rest.
  * remove Ninja as the default generator for ROS builds, as it breaks ros build farm builds
* Contributors: Meher Malladi

0.1.0 (2025-09-11)
------------------
* Bump version 0.1.0 (`#22 <https://github.com/PRBonn/rko_lio/issues/22>`_)
  * Update pyproject.toml to v0.1.0
  * Update package.xml to version 0.1.0
* Ros API improvements (`#20 <https://github.com/PRBonn/rko_lio/issues/20>`_)
  * append quat_xyzw_xyz to extrinsic parameter names to make the order clearer
  * rename bag_filename to bag_path to be more intuitive on the offline node
  * ros refactoring: more parameter definitions, better parameter defaults, better variable names, offline node simplifications, and readme and launfile updates to reflect the new parameter names.
  * update the rviz config file
  * launchfile is improved. cli params override config files. final config is printed to console
  * update the documentation to reflect the changes
* Contributors: Meher Malladi

0.0.4 (2025-09-09)
------------------
* C++ simplifications, py-rosbag reader improvements (`#14 <https://github.com/PRBonn/rko_lio/issues/14>`_)
  * print the names of the bag files being added if there are multiple
  * add a --dump_config option to get a default config out
  * improve the --help messages a bit
  * print a more helpful message in case the bag doesnt contain a tf
  * simplifications on the cpp side to how we compute interval stats for imu
  * better handle the case when there are no timestamps in the point cloud
  for a rosbag
  * change the names of a few variables
  * add some more details to the main readme
  * bump version
* Contributors: Meher Malladi

0.0.3 (2025-09-07 16:17)
------------------------
* Fix/improve rerun visualization, remove matplotlib as an undocumented dep, bump version (`#10 <https://github.com/PRBonn/rko_lio/issues/10>`_)
  * remove unnecessary dependency on matplotlib. also prevents a break
  * use the rerun blueprint from the visualization we have now
  * move rerun config to the package folder, and then always log it on
  initialization
  so we get a well configured viz by default
  * improve docs a bit, and change the name for a workflow build
  * bump version because we fixed the visualization in this one
* Add humble support and improve workflows (`#9 <https://github.com/PRBonn/rko_lio/issues/9>`_)
  * fix the problem with bag timestamp on humble
  * use my humble image for humble build
  * add details for humble in readmes. switch workflows branch back to
  master
  * split workflows into reusable workflows, so that i get badges on the
  readme
  * finish the quest for pretty status badges
* Contributors: Meher Malladi

0.0.2 (2025-09-07 03:17)
------------------------
* Add macOS and Windows support/builds/wheels (`#8 <https://github.com/PRBonn/rko_lio/issues/8>`_)
  * builds on mac arm (14 and 15)
  * builds on windows 2022
  * fix the msvc build error on helipr
  * added in tests
  * use pytests in the python bindings workflow
  * drop support for py 3.9
  * we provide wheels for all major platforms now
  * except windows 11 arm
  * version bump to 0.0.2
* Contributors: Meher Malladi

0.0.1 (2025-09-06)
------------------
* Workflows (`#6 <https://github.com/PRBonn/rko_lio/issues/6>`_)
  * test python bindings build workflow
  * add ros build workflow
  * update readmes to say we support kilted because it builds there as well
  * simplify cibuildwheel target platforms
  * add a pypi workflow. switch the branches out to master
  time to pray this works
* Add build and config docs (`#2 <https://github.com/PRBonn/rko_lio/issues/2>`_)
  * add build.md
  * move config and build docs into docs
  * add config.md
  * fix the gitignore mistake for folders
  * add a few more details
  * some ros doc cleanup
* Add initial readme documentation for both the ros and python versions (`#1 <https://github.com/PRBonn/rko_lio/issues/1>`_)
  * add some python readme docs
  * add a pre-commit config for fixing trailing whitespace
  * fix the math in the main readme
  * improve a link to python doc
  * add ros readme and add some placeholder details to the build and config
  * update the readmes a bit
  * add an example ros offline invocation
* add initial version
* Contributors: Meher Malladi
