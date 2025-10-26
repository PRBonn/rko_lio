import pytest

from rko_lio.lio import LIO, LIOConfig


def test_lioconfig_creation():
    config = LIOConfig()
    assert config is not None


def test_lioconfig_attributes():
    config = LIOConfig(max_range=50.0)
    config.voxel_size = 2.0
    config.deskew = False

    assert config.voxel_size == 2.0
    assert config.max_range == 50.0
    assert config.deskew is False


def test_lio_init_with_config():
    config = LIOConfig()
    lio_obj = LIO(config)
    assert lio_obj is not None
