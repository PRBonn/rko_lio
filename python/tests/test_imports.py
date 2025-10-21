import pytest


def test_package_importable():
    import rko_lio

    assert rko_lio is not None


def test_rko_lio_pybind_import():
    from rko_lio import rko_lio_pybind

    assert rko_lio_pybind is not None
