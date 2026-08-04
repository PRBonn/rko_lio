# all hail the power of vibe coding

import sys
import time

import rclpy
import rosbag2_py
import tf2_ros
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message

IMU_TYPE = "sensor_msgs/msg/Imu"
LIDAR_TYPE = "sensor_msgs/msg/PointCloud2"
BASE_FRAME_CANDIDATES = ("base_link", "base_footprint", "base")
AUTODETECTED = ("imu_topic", "lidar_topic", "imu_frame", "lidar_frame", "base_frame")
TF_SCAN_WINDOW_NS = 5 * 10**9


class AutodetectError(Exception):
    def __init__(self, message, param=None):
        super().__init__(message)
        self.param = param


def tf_frames(buffer):
    tree = yaml.safe_load(buffer.all_frames_as_yaml()) or {}
    return set(tree) | {entry["parent"] for entry in tree.values() if isinstance(entry, dict) and entry.get("parent")}


class LiveGraph:
    def __init__(self, node, buffer, executor, timeout):
        self.node = node
        self.buffer = buffer
        self.executor = executor
        self.timeout = timeout

    def spin_until(self, predicate, what):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            result = predicate()
            if result:
                return result
        raise AutodetectError(
            f"timed out after {self.timeout:.0f}s waiting for {what}. "
            "Is the data flowing? Raise autodetect_timeout, or pass the values explicitly."
        )

    def topics(self, msgtype):
        return self.spin_until(
            lambda: sorted(
                name
                for name, types in self.node.get_topic_names_and_types()
                if msgtype in types and not any(part.startswith("_") for part in name.split("/"))
            ),
            f"a {msgtype} publisher",
        )

    def frame_id(self, topic, msgtype):
        received = []
        subscription = self.node.create_subscription(
            get_message(msgtype),
            topic,
            lambda msg: received.append(msg.header.frame_id),
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            ),
        )
        try:
            return self.spin_until(lambda: received[0] if received else None, f"a message on {topic}")
        finally:
            self.node.destroy_subscription(subscription)

    def frames(self):
        return self.spin_until(lambda: tf_frames(self.buffer), "a TF tree")


class BagGraph:
    def __init__(self, bag_path, buffer):
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag_path)),
                rosbag2_py.ConverterOptions("", ""),
            )
        except Exception as error:
            raise AutodetectError(f"could not read the bag at {bag_path}: {error}") from error
        self.types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        self.buffer = buffer
        self.frame_of = {}

        wanted = {topic for topic, type_ in self.types.items() if type_ in (IMU_TYPE, LIDAR_TYPE)}
        tf_topics = {topic for topic, type_ in self.types.items() if type_ == "tf2_msgs/msg/TFMessage"}
        start = None
        while reader.has_next() and (wanted or tf_topics):
            topic, data, stamp = reader.read_next()
            start = stamp if start is None else start
            if stamp - start > TF_SCAN_WINDOW_NS:
                tf_topics.clear()
            if topic in tf_topics:
                for transform in deserialize_message(data, get_message(self.types[topic])).transforms:
                    self.buffer.set_transform_static(transform, "rko_lio_autodetect")
            elif topic in wanted:
                message = deserialize_message(data, get_message(self.types[topic]))
                self.frame_of[topic] = message.header.frame_id
                wanted.discard(topic)

    def topics(self, msgtype):
        return sorted(topic for topic, type_ in self.types.items() if type_ == msgtype)

    def frame_id(self, topic, msgtype):
        if topic not in self.frame_of:
            raise AutodetectError(f"the bag holds no messages on {topic}")
        return self.frame_of[topic]

    def frames(self):
        return tf_frames(self.buffer)


def pick_topic(graph, msgtype, argument):
    candidates = graph.topics(msgtype)
    if not candidates:
        raise AutodetectError(f"found no {msgtype} topic to use for {argument}", argument)
    if len(candidates) > 1:
        raise AutodetectError(
            f"found several {msgtype} topics, so {argument} cannot be guessed.\n"
            "Pass one of:\n" + "\n".join(f"  - {c}" for c in candidates),
            argument,
        )
    return candidates[0]


def resolve(graph, params):
    found = {}

    imu_topic = params.get("imu_topic")
    if not imu_topic:
        imu_topic = pick_topic(graph, IMU_TYPE, "imu_topic")
        found["imu_topic"] = imu_topic

    lidar_topic = params.get("lidar_topic")
    if not lidar_topic:
        lidar_topic = pick_topic(graph, LIDAR_TYPE, "lidar_topic")
        found["lidar_topic"] = lidar_topic

    imu_frame = params.get("imu_frame")
    if not imu_frame:
        imu_frame = graph.frame_id(imu_topic, IMU_TYPE)
        found["imu_frame"] = imu_frame

    lidar_frame = params.get("lidar_frame")
    if not lidar_frame:
        lidar_frame = graph.frame_id(lidar_topic, LIDAR_TYPE)
        found["lidar_frame"] = lidar_frame

    known_frames = graph.frames()
    base_frame = params.get("base_frame")
    if not base_frame:
        guessed = next((f for f in BASE_FRAME_CANDIDATES if f in known_frames), None)
        base_frame = guessed or lidar_frame
        found["base_frame"] = base_frame
        if not guessed and "invert_odom_tf" not in params:
            found["invert_odom_tf"] = True

    for name, frame in (("imu_frame", imu_frame), ("lidar_frame", lidar_frame)):
        if frame == base_frame:
            continue
        if not graph.buffer.can_transform(base_frame, frame, Time()):
            raise AutodetectError(
                f"the TF tree has no transform between {frame} and the base frame "
                f"{base_frame}, which the odometry needs.\n"
                "Known frames:\n"
                + "\n".join(f"  - {f}" for f in sorted(known_frames) or ["<none>"])
                + f"\nPublish the transform, override {name}, or give the extrinsics "
                "in a config file.",
                "base_frame",
            )
    return found


def autodetect_or_exit(params, mode, bag_path, timeout):
    if all(params.get(name) for name in AUTODETECTED):
        return params
    if mode == "offline" and not bag_path:
        return params

    context = rclpy.Context()
    rclpy.init(context=context, args=[])
    try:
        node = rclpy.create_node("rko_lio_autodetect", context=context)
        buffer = tf2_ros.Buffer()
        try:
            if mode == "offline":
                graph = BagGraph(bag_path, buffer)
            else:
                executor = SingleThreadedExecutor(context=context)
                executor.add_node(node)
                graph = LiveGraph(node, buffer, executor, timeout)
                # held only to keep the listener alive, it stops filling the buffer if collected
                _listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)
            found = resolve(graph, params)
        except AutodetectError as error:
            print("\n" + "=" * 40)
            print("[ERROR] autodetect failed:")
            print(error)
            hint = f" as {error.param}:=<value>" if error.param else ""
            print(f"Pass the values explicitly{hint}, or set autodetect:=false.")
            print("=" * 40 + "\n")
            sys.exit(1)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)

    if found:
        print("\nAutodetected:")
        for name, value in found.items():
            print(f"    {name}: {value}")
    return {**params, **found}
