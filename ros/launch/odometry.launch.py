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

offline_only_parameters = [
    {
        "name": "bag_path",
        "default": "",
        "description": "ROS bag path to process",
    },
    {
        "name": "skip_to_time",
        "default": "0.0",
        "description": "Skip till this timestamp in the bag",
    },
]

configurable_parameters = [
    {
        "name": "log_level",
        "default": "info",
        "description": "debug log level [DEBUG|INFO|WARN|ERROR|FATAL]",
    },
    {
        "name": "mode",
        "default": "online",
        "description": "Which node to launch: 'offline' or 'online'",
    },
    {
        "name": "config_file",
        "default": "",
        "description": "yaml config file",
    },
    {
        "name": "run_name",
        "default": "rko_lio_odometry_run",
        "description": "Name for this run",
    },
    # topics and frames
    {
        "name": "imu_topic",
        "default": "",
        "description": "IMU input topic",
    },
    {
        "name": "lidar_topic",
        "default": "",
        "description": "LiDAR pointcloud topic",
    },
    {
        "name": "imu_frame",
        "default": "",
        "description": "IMU frame",
    },
    {
        "name": "lidar_frame",
        "default": "",
        "description": "LiDAR frame",
    },
    {
        "name": "base_frame",
        "default": "",
        "description": "Estimation base frame name. Not necessarily the actual robot base frame",
    },
    {
        "name": "odom_frame",
        "default": "",
        "description": "World-fixed frame name",
    },
    {
        "name": "odom_topic",
        "default": "",
        "description": "name of the topic on which the Odom message gets published",
    },
    {
        "name": "invert_odom_tf",
        "default": "true",
        "description": "Whether to invert odometry transform",
    },
    {
        "name": "publish_local_map",
        "default": "false",
        "description": "Publish the local map that the odometry maintains",
    },
    {
        "name": "map_topic",
        "default": "",
        "description": "name of the topic on which the local map will be published if enabled",
    },
    {
        "name": "publish_map_after",
        "default": "1.0",
        "description": "seconds between each local map publish",
    },
    # Processing parameters
    {
        "name": "deskew",
        "default": "true",
        "description": "Deskew the point cloud before registration",
    },
    {
        "name": "voxel_size",
        "default": "1.0",
        "description": "Voxel grid filter size (meters)",
    },
    {
        "name": "max_points_per_voxel",
        "default": "20",
        "description": "Max points per voxel",
    },
    {
        "name": "max_range",
        "default": "100.0",
        "description": "Maximum valid LiDAR range (meters)",
    },
    {
        "name": "min_range",
        "default": "1.0",
        "description": "Minimum valid LiDAR range (meters)",
    },
    {
        "name": "max_correspondance_distance",
        "default": "0.5",
        "description": "ICP correspondence threshold (meters)",
    },
    {
        "name": "convergence_criterion",
        "default": "0.00001",
        "description": "Optimization stopping condition",
    },
    {
        "name": "max_num_threads",
        "default": "0",
        "description": "Number of threads for data association in ICP",
    },
    {
        "name": "publish_deskewed_scan",
        "default": "false",
        "description": "Publish the deskewed cloud, usually for visualizing",
    },
    {
        "name": "rviz",
        "default": "false",
        "description": "Launch rviz simultaneously",
    },
    {"name": "rviz_config_file", "default": "", "description": "rviz config file"},
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


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower()
    config_file = LaunchConfiguration("config_file").perform(context)
    params_from_file = {} if config_file == "" else yaml_to_dict(config_file)
    pkg_share_dir = get_package_share_directory("rko_lio")

    rviz_config_file = Path(LaunchConfiguration("rviz_config_file").perform(context))
    if rviz_config_file == Path(""):
        rviz_config_file = Path(pkg_share_dir) / "config" / "default.rviz"

    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower() == "true"

    # Prepare parameters
    params = set_configurable_parameters(configurable_parameters)
    # Remove offline-only parameters if in online mode
    if mode == "online":
        for p in ["bag_path", "skip_to_time"]:
            if p in params or p in params_from_file:
                print(
                    "WARNING: Running the online node, bag filename and skip_to_time are being ignored."
                )
            params.pop(p, None)
            params_from_file.pop(p, None)

    # final params with preference to params in the file, TODO: improve this
    parameters_list = [
        params,
        params_from_file,
    ]
    if rviz_enabled:
        parameters_list.append({"publish_deskewed_scan": True})

    node_executable = "online_node" if mode == "online" else "offline_node"

    nodes = [
        launch_ros.actions.Node(
            package="rko_lio",
            executable=node_executable,
            parameters=parameters_list,
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
