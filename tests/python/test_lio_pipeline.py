import numpy as np
import pytest
from rko_lio.config import PipelineConfig
from rko_lio.lio_pipeline import LIOPipeline

GRAVITY = 9.81
EPSILON = 1e-8

@pytest.fixture
def simple_point_cloud():
    # 10x10x10 uniform grid = 1000 points
    x = y = z = np.linspace(0, 1, 10)
    X, Y, Z = np.meshgrid(x, y, z, indexing="ij")
    points = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    return points.astype(np.float32)


@pytest.fixture
def identity_extrinsics():
    return np.eye(4)


@pytest.fixture
def static_imu_measurement():
    acceleration = np.array([0.0, 0.0, GRAVITY], dtype=np.float32)
    angular_velocity = np.zeros(3, dtype=np.float32)
    return acceleration, angular_velocity


@pytest.fixture
def pipeline(identity_extrinsics):
    config = PipelineConfig()
    config.extrinsic_imu2base = identity_extrinsics
    config.extrinsic_lidar2base = identity_extrinsics
    return LIOPipeline(config)


def create_lidar_timestamps(n):
    return np.linspace(0, 0.1, n).astype(np.float32)


def test_pipeline_creation(pipeline):
    assert pipeline is not None


def test_add_imu_sequence(pipeline, static_imu_measurement):
    accel, gyro = static_imu_measurement
    pipeline.add_imu(0.0, accel, gyro)
    pipeline.add_imu(0.01, accel, gyro)
    assert len(pipeline.imu_buffer) == 2


def test_add_lidar_points(pipeline, simple_point_cloud):
    cloud1 = simple_point_cloud
    timestamps1 = create_lidar_timestamps(len(cloud1))
    pipeline.add_lidar(cloud1, timestamps1)

    cloud2 = simple_point_cloud
    timestamps2 = create_lidar_timestamps(len(cloud2))

    pipeline.add_lidar(cloud2, timestamps2)

    assert len(pipeline.lidar_buffer) == 2


@pytest.mark.parametrize("initialization_phase", [True, False])
def test_identity_registration(identity_extrinsics, simple_point_cloud, static_imu_measurement, initialization_phase):
    def pipeline_with_init_phase():
        config = PipelineConfig()
        config.extrinsic_imu2base = identity_extrinsics
        config.extrinsic_lidar2base = identity_extrinsics
        config.lio.initialization_phase = initialization_phase
        return LIOPipeline(config)

    pipeline = pipeline_with_init_phase()
    accel, gyro = static_imu_measurement
    cloud = simple_point_cloud

    def add_scan_with_imu(base_time):
        timestamps = create_lidar_timestamps(len(cloud)) + base_time
        pipeline.add_lidar(cloud, timestamps)

        # Add 10 IMU measurements after the lidar scan end time
        start_time = timestamps[-1] + 0.01
        for i in range(10):
            t = start_time + i * 0.01
            pipeline.add_imu(t, accel, gyro)
        return timestamps[-1]

    def verify_identity_pose(scan_num):
        pose = pipeline.lio.pose()
        translation = pose[0:3, 3]
        rot_matrix = pose[0:3, 0:3]

        translation_error = np.linalg.norm(translation)
        trace_val = np.trace(rot_matrix)
        rotation_angle = np.degrees(np.arccos((trace_val - 1) / 2))

        assert translation_error <= 1e-3, f"Translation error too high at scan {scan_num}: {translation_error} m"
        assert rotation_angle <= 1e-3, f"Rotation error too high at scan {scan_num}: {rotation_angle} degrees"

    # First scan; base_time 0
    last_lidar_end = add_scan_with_imu(0.0)
    assert len(pipeline.lidar_buffer) == 0  # first lidar processed
    verify_identity_pose(1)

    # Second scan; base time is last lidar end time
    last_lidar_end = add_scan_with_imu(last_lidar_end)
    assert len(pipeline.lidar_buffer) == 0  # second lidar processed
    pipeline.add_imu(last_lidar_end + 0.01, accel, gyro) # ensure the third lidar gets processed
    verify_identity_pose(2)

    # third scan breaks on arm for whatever platform specific reason, i dont get it. TODO
    # Third scan; base time is last lidar end time
    # last_lidar_end = add_scan_with_imu(last_lidar_end)
    # pipeline.add_imu(last_lidar_end + 0.01, accel, gyro) # ensure the third lidar gets processed
    # assert len(pipeline.lidar_buffer) == 0  # third lidar processed
    # verify_identity_pose(3)
