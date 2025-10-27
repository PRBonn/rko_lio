import numpy as np
import pytest
from rko_lio.config import PipelineConfig
from rko_lio.lio_pipeline import LIOPipeline


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
def pipeline(identity_extrinsics):
    config = PipelineConfig()
    config.extrinsic_imu2base = identity_extrinsics
    config.extrinsic_lidar2base = identity_extrinsics
    return LIOPipeline(config)


def create_lidar_timestamps(n):
    return np.linspace(0, 0.1, n).astype(np.float32)


def test_pipeline_creation(pipeline):
    assert pipeline is not None


def test_add_imu_sequence(pipeline):
    pipeline.add_imu(0.0, np.zeros(3), np.zeros(3))
    pipeline.add_imu(0.01, np.zeros(3), np.zeros(3))
    assert len(pipeline.imu_buffer) == 2


def test_add_lidar_points(pipeline, simple_point_cloud):
    cloud1 = simple_point_cloud
    timestamps1 = create_lidar_timestamps(len(cloud1))
    pipeline.add_lidar(cloud1, timestamps1)

    cloud2 = simple_point_cloud
    timestamps2 = create_lidar_timestamps(len(cloud2))

    pipeline.add_lidar(cloud2, timestamps2)

    assert len(pipeline.lidar_buffer) == 2


def test_add_lidar_points_with_imu(pipeline, simple_point_cloud):
    cloud1 = simple_point_cloud
    timestamps1 = create_lidar_timestamps(len(cloud1))
    pipeline.add_lidar(cloud1, timestamps1)

    # Add 10 IMU measurements with increasing timestamps > lidar end time
    start_time = timestamps1[-1] + 0.01
    for i in range(10):
        t = start_time + i * 0.01
        pipeline.add_imu(t, np.zeros(3), np.zeros(3))

    assert len(pipeline.lidar_buffer) == 0  # first lidar processed for initialization

    cloud2 = simple_point_cloud
    timestamps2 = create_lidar_timestamps(len(cloud2)) + timestamps1[-1]
    pipeline.add_lidar(cloud2, timestamps2)

    # Add one imu measurement after second lidar end time just in case to trigger second register
    pipeline.add_imu(timestamps2[-1] + 0.01, np.zeros(3), np.zeros(3))

    assert len(pipeline.lidar_buffer) == 0  # second lidar processed

    pose = pipeline.lio.pose()
    assert np.allclose(pose, np.eye(4), atol=1e-6)
