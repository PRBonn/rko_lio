import pytest


def test_helipr_pybind_import():
    from rko_lio.dataloaders import helipr_file_reader_pybind

    assert hasattr(helipr_file_reader_pybind, "read_lidar_bin")


def test_helipr_import():
    from rko_lio.dataloaders.helipr import HeliprDataLoader

    assert HeliprDataLoader is not None
