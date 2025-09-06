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
