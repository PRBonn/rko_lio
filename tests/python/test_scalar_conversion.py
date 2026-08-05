"""Numpy <-> C++ conversion behaviour for the point vector binding.

The copy semantics are a performance contract: reading a vector back into numpy must stay a
view, and handing numpy in must leave the C++ side independent of the caller's buffer.
"""

import numpy as np
import pytest

from rko_lio import rko_lio_pybind as pybind

INPUT_DTYPES = [np.float32, np.float64]


def binding_dtype():
    """The scalar this package was built with, as numpy sees it."""
    return np.asarray(pybind._Vector3sVector(np.zeros((1, 3)))).dtype


def make_points(dtype, n=8):
    return (np.arange(n * 3, dtype=np.float64).reshape(n, 3) + 0.5).astype(dtype)


def test_binding_scalar_is_a_float_type():
    assert binding_dtype() in (np.dtype(np.float32), np.dtype(np.float64))


@pytest.mark.parametrize("dtype", INPUT_DTYPES)
def test_accepts_input_and_exposes_the_build_scalar(dtype):
    points = make_points(dtype)
    vector = pybind._Vector3sVector(points)

    assert len(vector) == len(points)
    assert np.asarray(vector).dtype == binding_dtype()
    np.testing.assert_allclose(np.asarray(vector), points.astype(binding_dtype()))


@pytest.mark.parametrize("dtype", INPUT_DTYPES)
def test_conversion_is_exact_when_no_narrowing_is_required(dtype):
    """Widening or an already matching dtype must be bit exact."""
    points = make_points(dtype)
    result = np.asarray(pybind._Vector3sVector(points))

    if np.can_cast(dtype, binding_dtype()):
        np.testing.assert_array_equal(result, points.astype(binding_dtype()))
    else:
        np.testing.assert_allclose(result, points, rtol=1e-6)


@pytest.mark.parametrize("dtype", INPUT_DTYPES)
def test_reading_back_is_a_view_not_a_copy(dtype):
    """C++ -> python must stay zero copy via the buffer protocol."""
    vector = pybind._Vector3sVector(make_points(dtype))
    view = np.asarray(vector)

    assert view.base is not None, "numpy copied instead of viewing the C++ storage"
    view[0, 0] = 123.0
    assert np.asarray(vector)[0, 0] == 123.0, "write through the view did not reach C++"


@pytest.mark.parametrize("dtype", INPUT_DTYPES)
def test_construction_copies_so_the_vector_owns_its_points(dtype):
    """python -> C++ takes a copy; a std::vector cannot alias the numpy buffer."""
    points = make_points(dtype)
    vector = pybind._Vector3sVector(points)

    points[0, 0] = -999.0
    assert np.asarray(vector)[0, 0] != -999.0, "vector aliased the caller's numpy buffer"


@pytest.mark.parametrize("dtype", INPUT_DTYPES)
def test_non_contiguous_input_is_handled(dtype):
    """Every other row of a bigger array is not c-contiguous; forcecast must fix it up."""
    points = make_points(dtype, n=16)[::2]

    assert not points.flags["C_CONTIGUOUS"]
    np.testing.assert_allclose(np.asarray(pybind._Vector3sVector(points)), points)


@pytest.mark.parametrize("dtype", INPUT_DTYPES)
def test_read_only_input_is_accepted(dtype):
    points = make_points(dtype)
    points.flags.writeable = False

    np.testing.assert_allclose(np.asarray(pybind._Vector3sVector(points)), points)


@pytest.mark.parametrize("dtype", INPUT_DTYPES)
def test_empty_input(dtype):
    assert len(pybind._Vector3sVector(np.zeros((0, 3), dtype=dtype))) == 0


@pytest.mark.parametrize("shape", [(4,), (4, 2), (4, 4), (2, 3, 3)])
def test_wrong_shape_is_rejected(shape):
    with pytest.raises((TypeError, ValueError, RuntimeError)):
        pybind._Vector3sVector(np.zeros(shape, dtype=np.float64))
