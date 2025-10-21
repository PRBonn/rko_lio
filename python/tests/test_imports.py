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
    from rko_lio.dataloaders.helipr import HeliprDataloader

    assert HeliprDataloader is not None


@pytest.mark.skipif(
    not pytest.importorskip(
        "open3d", reason="Optional dependency for raw dataloader (open3d) not installed"
    ),
)
def test_raw_import():
    from rko_lio.dataloaders.raw import RawDataLoader

    assert RawDataLoader is not None


@pytest.mark.skipif(
    not pytest.importorskip(
        "rosbags",
        reason="Optional dependency for rosbag dataloader (rosbags) not installed",
    ),
)
def test_rosbag_import():
    from rko_lio.dataloaders.rosbag import RosbagDataLoader

    assert RosbagDataLoader is not None
