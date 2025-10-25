import pytest


def test_raw_import():
    _ = pytest.importorskip(
        "open3d",
        reason="Optional dependency for raw dataloader (open3d) not installed",
    )
    from rko_lio.dataloaders.raw import RawDataLoader

    assert RawDataLoader is not None
