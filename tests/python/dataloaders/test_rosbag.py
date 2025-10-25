import pytest


def test_rosbag_import():
    _ = pytest.importorskip(
        "rosbags",
        reason="Optional dependency for rosbag dataloader (rosbags) not installed",
    )
    from rko_lio.dataloaders.rosbag import RosbagDataLoader

    assert RosbagDataLoader is not None
