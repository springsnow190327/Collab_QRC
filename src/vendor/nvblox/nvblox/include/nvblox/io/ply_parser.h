/*
Copyright 2025 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#pragma once

#include <fstream>
#include <string>

#include "nvblox/core/color.h"
#include "nvblox/core/time.h"
#include "nvblox/core/types.h"
#include "nvblox/mesh/mesh.h"
#include "nvblox/sensors/pointcloud.h"

namespace nvblox {
namespace io {
/// Reads a mesh/pointcloud from a .ply file. For reference on the format, see:
///  http://paulbourke.net/dataformats/ply/
/// Usage: Construct with filename, check isValid(), then access data or
/// convert. Note: Currently only supports ASCII format PLY files. The file is
/// parsed automatically in the constructor.
class PlyParser {
 public:
  explicit PlyParser(const std::string& filename);

  ~PlyParser() { file_.close(); }

  /// Check if the file was successfully parsed
  bool isValid() const { return is_valid_; }

  /// Convert parsed data to a ColorMesh.
  /// Returns true on success (if mesh data was available), false otherwise.
  bool toMesh(ColorMesh* mesh, const CudaStream& cuda_stream) const;

  /// Convert parsed data to a Pointcloud.
  /// Returns true on success (if point data was available), false otherwise.
  bool toPointcloud(Pointcloud* pointcloud,
                    const CudaStream& cuda_stream) const;

  // Accessors to get parsed data
  const std::vector<Vector3f>& points() const { return points_; }
  const std::vector<Color>& colors() const { return colors_; }
  const std::vector<float>& intensities() const { return intensities_; }
  const std::vector<Time>& timestamps_ms() const { return timestamps_ms_; }
  const std::vector<Vector3f>& normals() const { return normals_; }
  const std::vector<int>& triangles() const { return triangles_; }

  // Query methods to check what properties are available in the file
  bool hasNormals() const { return has_normals_; }
  bool hasColors() const { return has_colors_; }
  bool hasIntensities() const { return has_intensities_; }
  bool hasTimestamps() const { return has_timestamps_; }
  bool hasTriangles() const { return has_triangles_; }
  size_t numVertices() const { return num_vertices_; }
  size_t numFaces() const { return num_faces_; }

 private:
  bool parse();
  bool readHeader();
  void parseVertexProperties(const std::string& property_name,
                             int property_idx);
  bool readVertices();
  bool readFaces();

  // Parsed data
  std::vector<Vector3f> points_;
  std::vector<Vector3f> normals_;
  std::vector<float> intensities_;
  std::vector<Time> timestamps_ms_;
  std::vector<Color> colors_;
  std::vector<int> triangles_;

  std::ifstream file_;

  // Parsing state
  bool is_valid_ = false;

  // Header information
  size_t num_vertices_ = 0;
  size_t num_faces_ = 0;
  bool has_normals_ = false;
  bool has_colors_ = false;
  bool has_intensities_ = false;
  bool has_timestamps_ = false;
  bool has_triangles_ = false;

  // Property indices in the data stream
  int nx_idx_ = -1;
  int ny_idx_ = -1;
  int nz_idx_ = -1;
  int intensity_idx_ = -1;
  int timestamp_idx_ = -1;
  int red_idx_ = -1;
  int green_idx_ = -1;
  int blue_idx_ = -1;
};

}  // namespace io

}  // namespace nvblox
