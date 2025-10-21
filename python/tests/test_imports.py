from pathlib import Path

import pytest


def test_package_importable():
    import rko_lio

    assert rko_lio is not None


def test_rko_lio_pybind_import():
    from rko_lio import rko_lio_pybind

    assert rko_lio_pybind is not None


def test_helipr_pybind_import():
    from rko_lio.dataloaders import helipr_file_reader_pybind

    assert hasattr(helipr_file_reader_pybind, "read_lidar_bin")


def test_helipr_import():
    from rko_lio.dataloaders.helipr import HeliprDataLoader

    assert HeliprDataLoader is not None


def test_rosbag_import():
    _ = pytest.importorskip(
        "rosbags",
        reason="Optional dependency for rosbag dataloader (rosbags) not installed",
    )
    from rko_lio.dataloaders.rosbag import RosbagDataLoader

    assert RosbagDataLoader is not None


def test_raw_import():
    _ = pytest.importorskip(
        "open3d",
        reason="Optional dependency for raw dataloader (open3d) not installed",
    )
    from rko_lio.dataloaders.raw import RawDataLoader

    assert RawDataLoader is not None
