// ----------------------------------------------------------------------------
// NOTE: This file has been adapted from the Open3D project, but copyright
// still belongs to Open3D. All rights reserved
// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------
#pragma once
#include <Eigen/Core>
#include <vector>

// pollute namespace with py
#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace pybind11 {

template <typename Vector, typename holder_type = std::unique_ptr<Vector>, typename... Args>
py::class_<Vector, holder_type> bind_vector_without_repr(py::module& m, std::string const& name, Args&&... args) {
  // hack function to disable __repr__ for the convenient function
  // bind_vector()
  using Class_ = py::class_<Vector, holder_type>;
  Class_ cl(m, name.c_str(), std::forward<Args>(args)...);
  cl.def(py::init<>());
  cl.def("__bool__", [](const Vector& v) -> bool { return !v.empty(); }, "Check whether the list is nonempty");
  cl.def("__len__", &Vector::size);
  return cl;
}

// - This function is used by Pybind for std::vector<SomeEigenType> constructor.
//   This optional constructor is added to avoid too many Python <-> C++ API
//   calls when the vector size is large using the default binding method.
// - The numpy dtype follows the eigen vector's scalar, so a matching array is taken
//   straight through and anything else is converted once by forcecast.
template <typename EigenVector>
std::vector<EigenVector>
py_array_to_vectors(const py::array_t<typename EigenVector::Scalar, py::array::c_style | py::array::forcecast>& array) {
  const int64_t eigen_vector_size = EigenVector::SizeAtCompileTime;
  if (array.ndim() != 2 || array.shape(1) != eigen_vector_size) {
    throw py::cast_error();
  }
  std::vector<EigenVector> eigen_vectors(array.shape(0));
  const auto array_unchecked = array.template unchecked<2>();
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
  for (auto i = 0; i < array_unchecked.shape(0); ++i) {
    eigen_vectors[i] = Eigen::Map<const EigenVector>(&array_unchecked(i, 0));
  }
  // NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
  return eigen_vectors;
}

} // namespace pybind11

template <typename EigenVector,
          typename Vector = std::vector<EigenVector>,
          typename holder_type = std::unique_ptr<Vector>,
          typename InitFunc>
py::class_<Vector, holder_type> pybind_eigen_vector_of_vector(py::module& m,
                                                              const std::string& bind_name,
                                                              const std::string& repr_name,
                                                              InitFunc init_func) {
  using Scalar = EigenVector::Scalar;
  auto vec =
      py::bind_vector_without_repr<std::vector<EigenVector>>(m, bind_name, py::buffer_protocol(), py::module_local());
  vec.def(py::init(init_func));
  vec.def_buffer([](std::vector<EigenVector>& v) -> py::buffer_info {
    const size_t rows = EigenVector::RowsAtCompileTime;
    return py::buffer_info(v.data(), sizeof(Scalar), py::format_descriptor<Scalar>::format(), 2, {v.size(), rows},
                           {sizeof(EigenVector), sizeof(Scalar)});
  });
  vec.def("__repr__", [repr_name](const std::vector<EigenVector>& v) {
    return repr_name + std::string(" with ") + std::to_string(v.size()) + std::string(" elements.\n") +
           std::string("Use numpy.asarray() to access data.");
  });
  vec.def("__copy__", [](std::vector<EigenVector>& v) { return std::vector<EigenVector>(v); });
  vec.def("__deepcopy__", [](std::vector<EigenVector>& v) { return std::vector<EigenVector>(v); });

  // py::detail must be after custom constructor
  using Class_ = py::class_<Vector, std::unique_ptr<Vector>>;
  py::detail::vector_if_copy_constructible<Vector, Class_>(vec);
  py::detail::vector_if_equal_operator<Vector, Class_>(vec);
  py::detail::vector_modifiers<Vector, Class_>(vec);
  py::detail::vector_accessor<Vector, Class_>(vec);

  return vec;
}
