# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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
import sys
from pathlib import Path

import numpy as np

from ..util import error, error_and_exit, info, warning

try:
    from rosbags.highlevel import AnyReader
except ModuleNotFoundError:
    error_and_exit(
        'rosbags library not installed for using rosbag dataloader, please install with "pip install -U rosbags"'
    )

from .. import rko_lio_pybind
from ..scoped_profiler import ScopedProfiler
from .utils.ros_read_point_cloud import read_point_cloud as ros_read_point_cloud
from .utils.static_tf_tree import create_static_tf_tree, query_static_tf


class RosbagDataLoader:
    def __init__(
        self,
        data_path: Path,
        imu_topic: str | None,
        lidar_topic: str | None,
        imu_frame_id: str | None,
        lidar_frame_id: str | None,
        base_frame_id: str | None,
        *args,
        **kwargs,
    ):
        """query_tf_tree: try to query a tf tree if it exists"""

        assert (
            data_path.is_dir()
        ), "Pass a directory to data_path with ros1 or ros2 bag files"

        self.rko_lio_pybind = rko_lio_pybind
        self.ros_read_point_cloud = ros_read_point_cloud

        ros1_bagfiles = sorted(list(data_path.glob("*.bag")))
        bagfiles = None
        if ros1_bagfiles:
            self.bag_type = "ROS1"  # for logging
            bagfiles = ros1_bagfiles
        else:
            self.bag_type = "ROS2"
            bagfiles = [data_path]
        self.first_bag_path = bagfiles[0]  # for logging
        self.bag = AnyReader(bagfiles)
        if len(bagfiles) > 1:
            print("Reading multiple .bag files in directory:")
            print("\n".join(sorted([path.name for path in bagfiles])))
        self.bag.open()

        self.lidar_topic = self.check_topic(
            lidar_topic, expected_msgtype="sensor_msgs/msg/PointCloud2"
        )
        self.imu_topic = self.check_topic(
            imu_topic, expected_msgtype="sensor_msgs/msg/Imu"
        )
        self.connections = [
            x
            for x in self.bag.connections
            if (x.topic == self.imu_topic or x.topic == self.lidar_topic)
        ]
        self.imu_frame_id = imu_frame_id or self._read_first_frame_id(self.imu_topic)
        self.lidar_frame_id = lidar_frame_id or self._read_first_frame_id(
            self.lidar_topic
        )
        self.base_frame_id = base_frame_id or self.lidar_frame_id

        if self.base_frame_id is None:
            error_and_exit(
                f"Could not automatically determine a base frame id. Please pass it with --base_frame."
            )

        self.T_imu_to_base = None
        self.T_lidar_to_base = None

        self.msgs = self.bag.messages(connections=self.connections)

    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def __len__(self):
        return (
            self.bag.topics[self.imu_topic].msgcount
            + self.bag.topics[self.lidar_topic].msgcount
        )

    def _read_first_frame_id(self, topic_name):
        """Read the frame_id from the first message of the given topic."""
        for connection in self.bag.connections:
            if connection.topic == topic_name:
                msg_iter = self.bag.messages(connections=[connection])
                _, _, rawdata = next(msg_iter)  # first message
                deserialized = self.bag.deserialize(rawdata, connection.msgtype)
                return deserialized.header.frame_id

    @property
    def extrinsics(self):
        if self.T_imu_to_base is None or self.T_lidar_to_base is None:
            info("Trying to obtain extrinsics from the data.")
            print("Building TF tree.")
            static_tf_tree = create_static_tf_tree(self.bag)
            if not static_tf_tree:
                error_and_exit(
                    "The rosbag doesn't contain a static tf tree, cannot query it for extrinsics. Please specify the extrinsics manually in a config. You can use 'rko_lio --dump_config' to dump a default config."
                )

            print("Querying TF tree for imu to base extrinsic.")
            self.T_imu_to_base = query_static_tf(
                static_tf_tree, self.imu_frame_id, self.base_frame_id
            )
            print("Querying TF tree for lidar to base extrinsic.")
            self.T_lidar_to_base = query_static_tf(
                static_tf_tree, self.lidar_frame_id, self.base_frame_id
            )
        return self.T_imu_to_base, self.T_lidar_to_base

    def __iter__(self):
        return self

    def __next__(self):
        while True:
            with ScopedProfiler("Rosbag Dataloader") as data_timer:
                connection, bag_timestamp, rawdata = next(self.msgs)
                deserialized_data = self.bag.deserialize(rawdata, connection.msgtype)
                if connection.topic == self.imu_topic:
                    return "imu", self.read_imu(deserialized_data)
                elif connection.topic == self.lidar_topic:
                    try:
                        return "lidar", self.read_point_cloud(deserialized_data)
                    except RuntimeError as e:
                        # the cpp side can throw on _process_timestamps
                        warning(
                            "Error processing lidar frame. You should probably report an issue."
                        )
                        continue
                raise NotImplementedError("Shouldn't happen.")

    def read_imu(self, data):
        header_stamp = data.header.stamp
        timestamp = header_stamp.sec + (header_stamp.nanosec / 1e9)
        gyro = [
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z,
        ]
        accel = [
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
        ]
        return timestamp, accel, gyro

    def read_point_cloud(self, data):
        header_stamp = data.header.stamp
        header_stamp_sec = header_stamp.sec + (header_stamp.nanosec / 1e9)
        points, raw_timestamps = self.ros_read_point_cloud(data)
        if raw_timestamps is not None and raw_timestamps.size > 0:
            _, _, abs_timestamps = self.rko_lio_pybind._process_timestamps(
                self.rko_lio_pybind._VectorDouble(raw_timestamps), header_stamp_sec
            )
            return points, np.asarray(abs_timestamps)
        else:
            raw_timestamps = np.ones(points.shape[0]) * header_stamp_sec
            if not hasattr(self, "_printed_timestamp_warning"):
                self._printed_timestamp_warning = True
                warning(
                    "Could not detect timestamps in the point cloud. Odometry performance will suffer. Also please disable deskewing (enabled by default) otherwise the odometry may not work properly."
                )
            return points, raw_timestamps

    def check_topic(self, topic: str | None, expected_msgtype: str) -> str:
        topics_of_type = [
            topic_name
            for topic_name, info in self.bag.topics.items()
            if info.msgtype == expected_msgtype
        ]

        def print_available_topics_and_exit():
            print(50 * "-")
            for t in topics_of_type:
                print(f"--{'imu' if expected_msgtype.endswith('Imu') else 'lidar'} {t}")
            print(50 * "-")
            sys.exit(1)

        if topic and topic in topics_of_type:
            return topic
        if topic and topic not in topics_of_type:
            error(
                "Rosbag does not contain any msg with the topic name",
                topic,
                ". Please select one of these for",
                expected_msgtype,
            )
            print_available_topics_and_exit()
        if len(topics_of_type) > 1:
            error(
                "Multiple",
                expected_msgtype,
                "topics available. Please select one with the appropriate flag.",
            )
            print_available_topics_and_exit()
        if len(topics_of_type) == 0:
            error_and_exit(
                "Your rosbag does not contain any", expected_msgtype, "topic."
            )
        return topics_of_type[0]

    def __repr__(self):
        bag_type = f"ros_version='{self.bag_type}'"
        path_info = f"bag_files='{self.first_bag_path}{'...' if self.bag_type == 'ROS1' else ''}'"
        imu_info = f"imu_topic='{self.imu_topic}'"
        lidar_info = f"lidar_topic='{self.lidar_topic}'"
        msg_counts = (
            f"{self.bag.topics[self.imu_topic].msgcount if self.imu_topic in self.bag.topics else 0} IMU msgs, "
            f"{self.bag.topics[self.lidar_topic].msgcount if self.lidar_topic in self.bag.topics else 0} LiDAR msgs"
        )
        return f"RosbagDataLoader({bag_type}, {path_info}, {imu_info}, {lidar_info}, {msg_counts})"
