/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// based on
// https://github.com/minwoo0611/HeLiPR-File-Player/blob/e8d95e390454ece1415ae9deb51515f63730c10a/src/ROSThread.cpp#L411
#include <filesystem>
#include <fstream>
#include <numeric>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace {
// Helper for aeva: get file stem as int64_t
int64_t bin_file_stem(const std::string& filename) {
  namespace fs = std::filesystem;
  fs::path p(filename);
  return std::stoll(p.stem().string());
}

enum class SensorType { Velodyne, Ouster, Avia, AevaOld, AevaNew };

SensorType parse_sensor_type_and_variant(const std::string& sensor, const std::string& filename) {
  if (sensor == "Velodyne") {
    return SensorType::Velodyne;
  }
  if (sensor == "Ouster") {
    return SensorType::Ouster;
  }
  if (sensor == "Avia") {
    return SensorType::Avia;
  }
  if (sensor == "Aeva") {
    const int64_t stem = bin_file_stem(filename);
    if (stem > 1691936557946849179LL) {
      return SensorType::AevaNew;
    } else {
      return SensorType::AevaOld;
    }
  }
  throw std::runtime_error("Unknown sensor: " + sensor);
}

// thanks to https://stackoverflow.com/a/79177816
// basically i dont want any padding on the structs due to alignment
// that much i get, but what do the msvc macros mean? i dont know man
// but really this is incredibly ugly, and i dont want to maintain it, helipr goes away in the first major release
#ifdef __GNUC__
#define PACK__
#define __PACK __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK__ __pragma(pack(push, 1))
#define __PACK __pragma(pack(pop))
#endif

PACK__ struct HeLiPR_velodyne {
  float x, y, z, intensity;
  uint16_t ring;
  float time;
} __PACK;

PACK__ struct HeLiPR_ouster {
  float x, y, z, intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint16_t ring;
  uint16_t ambient;
} __PACK;

PACK__ struct HeLiPR_avia {
  float x, y, z;
  uint8_t reflectivity, tag, line;
  uint32_t offset_time;
} __PACK;

PACK__ struct HeLiPR_aeva {
  float x, y, z, reflectivity, velocity;
  int32_t time_offset_ns;
  uint8_t line_index;
} __PACK;

PACK__ struct HeLiPR_aeva_new {
  float x, y, z, reflectivity, velocity;
  int32_t time_offset_ns;
  uint8_t line_index;
  float intensity;
} __PACK;

// makes a few things easier through templating
template <SensorType sensor_type>
struct SensorTraits;

template <>
struct SensorTraits<SensorType::Velodyne> {
  using Record = HeLiPR_velodyne;
  static float get_time(const Record& r) { return r.time; }
};

template <>
struct SensorTraits<SensorType::Ouster> {
  using Record = HeLiPR_ouster;
  static float get_time(const Record& r) { return static_cast<float>(r.t); }
};

template <>
struct SensorTraits<SensorType::Avia> {
  using Record = HeLiPR_avia;
  static float get_time(const Record& r) { return static_cast<float>(r.offset_time); }
};

template <>
struct SensorTraits<SensorType::AevaOld> {
  using Record = HeLiPR_aeva;
  static float get_time(const Record& r) { return static_cast<float>(r.time_offset_ns); }
};

template <>
struct SensorTraits<SensorType::AevaNew> {
  using Record = HeLiPR_aeva_new;
  static float get_time(const Record& r) { return static_cast<float>(r.time_offset_ns); }
};

using PointsAndTime = std::pair<std::vector<float>, std::vector<float>>;
template <SensorType sensor_type>
  requires requires { typename SensorTraits<sensor_type>::Record; }
PointsAndTime read_helipr_data(const std::string& filename) {
  using Traits = SensorTraits<sensor_type>;
  using Record = typename Traits::Record;

  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  file.seekg(0, std::ios::end);
  const auto file_size_bytes = static_cast<size_t>(file.tellg());
  if (file_size_bytes % sizeof(Record) != 0) {
    throw std::runtime_error("File size is not a multiple of record size");
  }
  file.seekg(0, std::ios::beg);

  const size_t record_count = file_size_bytes / sizeof(Record);
  std::vector<Record> records(record_count);

  // read the entire file at once
  file.read(reinterpret_cast<char*>(records.data()), file_size_bytes);
  if (!file) {
    throw std::runtime_error("Error reading file, possibly truncated: " + filename);
  }

  // can use Eigen::Map style things to prevent copies, but idc, this is getting deprecated later
  std::vector<float> points(record_count * 3);
  std::vector<float> times(record_count);

  std::vector<size_t> indices(record_count);
  std::iota(indices.begin(), indices.end(), 0);

  std::for_each(indices.cbegin(), indices.cend(), [&](const size_t i) {
    const auto& rec = records[i];
    points[3 * i] = rec.x;
    points[3 * i + 1] = rec.y;
    points[3 * i + 2] = rec.z;
    times[i] = Traits::get_time(rec);
  });

  return {points, times};
}

py::tuple read_lidar_bin(const std::string& filename, const std::string& sensor) {
  const SensorType sensor_type = parse_sensor_type_and_variant(sensor, filename);

  // Points will be x, y, z per point (row-major), so it can be reshaped on the python side.
  auto data = PointsAndTime{};
  switch (sensor_type) {
  case SensorType::Velodyne:
    data = read_helipr_data<SensorType::Velodyne>(filename);
    break;
  case SensorType::Ouster:
    data = read_helipr_data<SensorType::Ouster>(filename);
    break;
  case SensorType::Avia:
    data = read_helipr_data<SensorType::Avia>(filename);
    break;
  case SensorType::AevaOld:
    data = read_helipr_data<SensorType::AevaOld>(filename);
    break;
  case SensorType::AevaNew:
    data = read_helipr_data<SensorType::AevaNew>(filename);
    break;
  default:
    throw std::runtime_error("Unsupported sensor type");
  }
  return py::make_tuple(std::move(data.first), std::move(data.second));
}
} // namespace

PYBIND11_MAKE_OPAQUE(std::vector<float>);

PYBIND11_MODULE(helipr_file_reader_pybind, m) {
  py::bind_vector<std::vector<float>>(m, "_VectorFloat");
  m.def("read_lidar_bin", &read_lidar_bin, "Read a single HeLiPR lidar .bin file (sensor-specific)");
}
