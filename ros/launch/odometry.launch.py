# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from pathlib import Path

import launch_ros.actions
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# import logging
# logging.root.setLevel(logging.DEBUG)


offline_only_parameters = [
    {
        "name": "bag_path",
        "default": "",
        "description": "[Offline node only] ROS bag path to process (required)",
        "required": True,
    },
    {
        "name": "skip_to_time",
        "default": "0.0",
        "type": "float",
        "description": "[Offline node only] Skip to timestamp in the bag (seconds)",
    },
]

configurable_parameters = [
    {
        "name": "imu_topic",
        "default": "",
        "description": "IMU input topic (required)",
        "required": True,
    },
    {
        "name": "lidar_topic",
        "default": "",
        "description": "LiDAR pointcloud topic (required)",
        "required": True,
    },
    {
        "name": "imu_frame",
        "default": "",
        "description": "IMU frame, can be left empty if message frame id is to be used",
    },
    {
        "name": "lidar_frame",
        "default": "",
        "description": "LiDAR frame, can be left empty if message frame id is to be used",
    },
    {
        "name": "base_frame",
        "default": "",
        "description": "Robot base frame, or the frame of estimation for odometry (required)",
        "required": True,
    },
    {
        "name": "odom_frame",
        "default": "odom",
        "description": "Odom frame id, used for publishing the tf from base frame to odom frame",
    },
    {
        "name": "odom_topic",
        "default": "/rko_lio/odometry",
        "description": "Odometry topic name",
    },
    {
        "name": "invert_odom_tf",
        "default": "False",
        "type": "bool",
        "description": "Invert odometry transform if required so that the base frame is the parent and odom frame is the child in the TF tree",
    },
    {
        "name": "publish_local_map",
        "default": "False",
        "type": "bool",
        "description": "Publish local map",
    },
    {
        "name": "map_topic",
        "default": "/rko_lio/local_map",
        "description": "Local map topic. Published if publish_local_map is true",
    },
    {
        "name": "publish_map_after",
        "default": "1.0",
        "type": "float",
        "description": "Seconds between each local map publishing",
    },
    {
        "name": "publish_lidar_acceleration",
        "default": "False",
        "type": "bool",
        "description": "Publish the linear acceleration of the `base_frame` expressed in `base_frame` coordinates. Note that this acceleration can be quite noisy, as it is essentially a double time derivative of the pose update from the lidar scan registration (similar to the twist/velocity in the odometry topic). The topic name is /rko_lio/lidar_acceleration.",
    },
    # lio parameters
    {
        "name": "deskew",
        "default": "True",
        "type": "bool",
        "description": "Deskew the point cloud before registration, requires timestamps in the Lidar messages",
    },
    {
        "name": "voxel_size",
        "default": "1.0",
        "type": "float",
        "description": "Voxel size for the local map (meters) used for odometry",
    },
    {
        "name": "max_points_per_voxel",
        "default": "20",
        "type": "int",
        "description": "Max points per voxel",
    },
    {
        "name": "max_range",
        "default": "100.0",
        "type": "float",
        "description": "Max valid LiDAR range (meters)",
    },
    {
        "name": "min_range",
        "default": "1.0",
        "type": "float",
        "description": "Min valid LiDAR range (meters)",
    },
    {
        "name": "max_correspondance_distance",
        "default": "0.5",
        "type": "float",
        "description": "ICP correspondence distance threshold (meters)",
    },
    {
        "name": "convergence_criterion",
        "default": "0.00001",
        "type": "float",
        "description": "Optimization stopping condition for ICP",
    },
    {
        "name": "max_num_threads",
        "default": "0",
        "type": "int",
        "description": "Number of threads used for data association in ICP",
    },
    {
        "name": "publish_deskewed_scan",
        "default": "false",
        "type": "bool",
        "description": "Publish deskewed scan for visualization",
    },
    {
        "name": "initialization_phase",
        "default": "false",
        "type": "bool",
        "description": "Use the IMU data between the first two frames to initialize IMU bias and system orientation. Assumes the system is at rest between these two frames. WARNING: If this is enabled, but the odometry starts while the system is in motion, the odometry will likely not work as expected. But I highly recommended enabling this while ensuring the system starts from rest.",
    },
    {
        "name": "max_iterations",
        "default": "100",
        "type": "int",
        "description": "Max ICP iterations",
    },
    {
        "name": "max_expected_jerk",
        "default": "3.0",
        "type": "float",
        "description": "Max expected jerk (m/s^3)",
    },
    {
        "name": "double_downsample",
        "default": "true",
        "type": "bool",
        "description": "Double downsample the input cloud before registration. Useful for dense Lidars. Can be disabled for sparse lidars.",
    },
    {
        "name": "min_beta",
        "default": "200",
        "type": "float",
        "description": "Scale parameter that decides the minimum amount of orientation regularisation applied during ICP registration.",
    },
    # ros params
    {
        "name": "mode",
        "default": "online",
        "description": "Launch mode: 'offline' or 'online'",
    },
    {
        "name": "config_file",
        "default": "",
        "description": "YAML config file to load parameters from",
    },
    {
        "name": "run_name",
        "default": "rko_lio_odometry_run",
        "description": "Run name, used when saving results to results_dir",
    },
    {
        "name": "results_dir",
        "default": "results",
        "description": "When the odometry node exists, it dumps a trajectory file to this directory under a 'run_name' subfolder along with the LIO configuration parameters used for that run.",
    },
    {
        "name": "rviz",
        "default": "false",
        "type": "bool",
        "description": "Launch RViz simultaneously",
    },
    {
        "name": "rviz_config_file",
        "default": "config/default.rviz",
        "description": "RViz config file path. If it's not the default value, note that it will be passed to rviz as is.",
    },
    {
        "name": "log_level",
        "default": "info",
        "description": "ROS Log level [DEBUG|INFO|WARN|ERROR|FATAL]",
    },
] + offline_only_parameters


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param.get("description", ""),
        )
        for param in parameters
    ]


def auto_cast_params(params, param_defs):
    """
    Because for some reason, I cannot use a raw bool in configurable parameters dict
    because of how DeclareLaunchArgument seems to work, so i have to write it as a string.
    But then my node expects a bool so I need to explicitly cast it the string to bool again...
    There must be a simpler way to do all this...
    """
    out = {}
    for p in param_defs:
        name = p["name"]
        if name in params:
            v = params.get(name)
            tp = p.get("type", None)
            if tp == "bool":
                out[name] = str(v).lower() == "true"
            elif tp == "int":
                out[name] = int(v)
            elif tp == "float":
                out[name] = float(v)
            else:
                out[name] = v
    return out


def get_configured_cli_parameters(configurable_parameters, context):
    "Return only CLI parameters that were explicitly set by the user"
    explicit_params = {
        arg.split(":=")[0] for arg in getattr(context, "argv", []) if ":=" in arg
    }
    cli_params = {}
    for param in configurable_parameters:
        name = param["name"]
        if name in explicit_params:
            cli_params[name] = LaunchConfiguration(name).perform(context)
    return auto_cast_params(cli_params, configurable_parameters)


def get_config_file_parameters(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    if config_file == "":
        return {}
    with open(config_file, "r") as f:
        return yaml.safe_load(f)


def merge_and_validate_parameters(
    cli_params: dict,
    file_params: dict,
    mode: str,
) -> dict:
    """
    Merge CLI and file parameters:
      - CLI overrides file values only if non-empty
      - Validate required params are provided (depending on mode)
    """
    merged = {}

    merged.update(file_params)

    # override with cli only if non-empty
    for k, v in cli_params.items():
        if v not in ("", None):
            merged[k] = v

    # validating required params
    missing = []
    for param in configurable_parameters:
        name = param["name"]

        # skip offline-only params in online mode
        if mode == "online" and param in offline_only_parameters:
            continue

        if param.get("required", False):
            if name not in merged or not merged.get(name):
                missing.append(name)

    if missing:
        print("\n\n" + "=" * 40)
        print("[ERROR] missing required parameter(s):")
        print(", ".join(missing))
        print("Please provide them via cli (param:=value) or a config file.")
        print("=" * 40 + "\n\n")
        import sys

        sys.exit(1)

    # Check extrinsics
    extrinsic_params = [
        "extrinsic_lidar2base_quat_xyzw_xyz",
        "extrinsic_imu2base_quat_xyzw_xyz",
    ]
    extrinsic_set = [
        p in merged and merged[p] not in ("", None) for p in extrinsic_params
    ]
    if any(extrinsic_set) and not all(extrinsic_set):
        print("\n\n" + "=" * 40)
        print("[ERROR] extrinsic parameters incomplete:")
        print(
            "If one of {} is specified, both must be provided.".format(
                ", ".join(extrinsic_params)
            )
        )
        print(
            "Please provide them via a config file. If you only need one, then explicitly set the other to identity."
        )
        print("=" * 40 + "\n\n")
        import sys

        sys.exit(1)

    return merged


def prepare_rviz_config(rviz_config_file: Path, parameters: dict) -> Path:
    """
    Decide which RViz config to use.
    If rviz_config_file is not the default, just return it.
    Otherwise, patch the default config with base_frame.
    """
    default_config = Path("config/default.rviz")

    # If user provided a custom config, just return it unchanged
    if rviz_config_file != default_config:
        return rviz_config_file

    rviz_config_file = Path(get_package_share_directory("rko_lio")) / rviz_config_file

    base_frame = parameters.get("base_frame", "")
    odom_frame = parameters.get("odom_frame", "")
    if not base_frame and not odom_frame:
        return rviz_config_file  # no override needed

    # Load default config
    with open(rviz_config_file, "r") as f:
        rviz_cfg = yaml.safe_load(f)

    try:
        # Patch whichever frame is specified
        if base_frame:
            rviz_cfg["Visualization Manager"]["Views"]["Current"]["Target Frame"] = base_frame
        if odom_frame:
            rviz_cfg["Visualization Manager"]["Global Options"]["Fixed Frame"] = odom_frame
    except Exception as e:
        raise RuntimeError(
            f"Could not patch RViz config with frames (base_frame={base_frame}, odom_frame={odom_frame}): {e}"
        )

    # Write to a temp file
    import tempfile

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".rviz", delete=False)
    yaml.safe_dump(rviz_cfg, tmp)
    tmp.flush()
    # since its the default rviz config file, we also want to viz the deskewed scan and local map
    parameters["publish_deskewed_scan"] = True
    parameters["publish_local_map"] = True

    return Path(tmp.name)


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower()

    # Prepare parameters
    cli_params = get_configured_cli_parameters(configurable_parameters, context=context)
    params_from_file = get_config_file_parameters(context)
    final_params = merge_and_validate_parameters(
        cli_params=cli_params,
        file_params=params_from_file,
        mode=mode,
    )

    print("\n" + "=" * 40 + "\n")
    print("Using Launch configuration:\n")
    print(yaml.dump(final_params, sort_keys=False, default_flow_style=False, indent=4))
    print("=" * 40 + "\n")

    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower() == "true"
    if rviz_enabled:
        rviz_config_file = prepare_rviz_config(
            Path(LaunchConfiguration("rviz_config_file").perform(context)),
            final_params,
        )

    node_executable = "online_node" if mode == "online" else "offline_node"

    nodes = [
        launch_ros.actions.Node(
            package="rko_lio",
            executable=node_executable,
            parameters=[final_params],
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            emulate_tty=True,
        )
    ]

    if rviz_enabled:
        nodes.append(
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file.as_posix()],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters)
        + [OpaqueFunction(function=launch_setup)]
    )
