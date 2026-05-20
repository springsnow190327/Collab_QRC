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

#include "nvblox/io/ply_parser.h"

#include <sstream>

namespace nvblox {
namespace io {

PlyParser::PlyParser(const std::string& filename) : file_(filename) {
  if (!file_.is_open()) {
    LOG(ERROR) << "Failed to open PLY file: " << filename;
    is_valid_ = false;
    return;
  }

  // Parse the file automatically in the constructor
  is_valid_ = parse();
}

bool PlyParser::parse() {
  // Read and parse the header
  if (!readHeader()) {
    LOG(ERROR) << "Failed to parse PLY header.";
    return false;
  }

  // Reserve space in internal vectors
  points_.reserve(num_vertices_);
  if (has_normals_) {
    normals_.reserve(num_vertices_);
  }
  if (has_colors_) {
    colors_.reserve(num_vertices_);
  }
  if (has_intensities_) {
    intensities_.reserve(num_vertices_);
  }
  if (has_timestamps_) {
    timestamps_ms_.reserve(num_vertices_);
  }
  if (has_triangles_) {
    triangles_.reserve(num_faces_ * 3);
  }

  // Read vertex data
  if (!readVertices()) {
    LOG(ERROR) << "Failed to read vertex data.";
    return false;
  }

  // Read face/triangle data if present
  if (has_triangles_ && !readFaces()) {
    LOG(ERROR) << "Failed to read face data.";
    return false;
  }

  return true;
}

bool PlyParser::readHeader() {
  std::string line;

  // First line should be "ply"
  if (!std::getline(file_, line) || line.find("ply") == std::string::npos) {
    LOG(ERROR) << "Invalid PLY file: missing 'ply' header.";
    return false;
  }

  // Second line should be format
  if (!std::getline(file_, line) || line.find("ascii") == std::string::npos) {
    LOG(ERROR) << "Only ASCII format PLY files are supported.";
    return false;
  }

  // Parse the rest of the header
  int property_idx = 0;
  std::string current_element_type;

  while (std::getline(file_, line)) {
    std::istringstream iss(line);
    std::string token;
    iss >> token;

    if (token == "end_header") {
      break;
    } else if (token == "element") {
      // Parse the element
      iss >> current_element_type;
      if (current_element_type == "vertex") {
        iss >> num_vertices_;
      } else if (current_element_type == "face") {
        iss >> num_faces_;
        has_triangles_ = true;
      }
      // Reset property index at new element token
      property_idx = 0;
    } else if (token == "property") {
      // Parse the properties for the current element type
      std::string type, property_name;
      if (current_element_type == "vertex") {
        iss >> type >> property_name;
        parseVertexProperties(property_name, property_idx);
      } else if (current_element_type == "face") {
        iss >> type;
        if (type != "list") {
          LOG(ERROR) << "Expected 'list' identifier in face property, got '"
                     << type << "'";
          return false;
        }
      }
      property_idx++;
    }
  }

  // Sanity checks on parsed values to prevent memory allocation issues
  constexpr size_t kMaxVertices = 1e7;
  if (num_vertices_ == 0 || num_vertices_ > kMaxVertices) {
    LOG(ERROR)
        << "Number of vertices in header invalid or exceeds maximum allowed ("
        << num_vertices_ << ")";
    return false;
  }

  constexpr size_t kMaxFaces = 1e6;
  if (num_faces_ > kMaxFaces) {
    LOG(ERROR) << "Number of faces in header exceeds maximum allowed ("
               << num_faces_ << ")";
    return false;
  }

  return true;
}

void PlyParser::parseVertexProperties(const std::string& property_name,
                                      int property_idx) {
  // Skip x, y, z (always at indices 0, 1, 2)
  if (property_name == "x" || property_name == "y" || property_name == "z") {
    return;
  }

  // Track normal properties
  if (property_name == "nx" || property_name == "normal_x") {
    has_normals_ = true;
    nx_idx_ = property_idx;
  } else if (property_name == "ny" || property_name == "normal_y") {
    ny_idx_ = property_idx;
  } else if (property_name == "nz" || property_name == "normal_z") {
    nz_idx_ = property_idx;
  }
  // Track intensity
  else if (property_name == "intensity") {
    has_intensities_ = true;
    intensity_idx_ = property_idx;
  }
  // Track timestamp
  else if (property_name == "t") {
    has_timestamps_ = true;
    timestamp_idx_ = property_idx;
  }
  // Track color properties
  else if (property_name == "red" || property_name == "r") {
    has_colors_ = true;
    red_idx_ = property_idx;
  } else if (property_name == "green" || property_name == "g") {
    green_idx_ = property_idx;
  } else if (property_name == "blue" || property_name == "b") {
    blue_idx_ = property_idx;
  }
}

bool PlyParser::readVertices() {
  std::string line;

  for (size_t i = 0; i < num_vertices_; i++) {
    if (!std::getline(file_, line)) {
      LOG(ERROR) << "Unexpected end of file at vertex " << i;
      return false;
    }

    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;

    // Read all tokens from the line
    while (iss >> token) {
      tokens.push_back(token);
    }

    // Need at least x, y, z
    if (tokens.size() < 3) {
      LOG(ERROR) << "Invalid vertex data at line " << i;
      return false;
    }

    // Read position (always first three values)
    Vector3f point;
    point.x() = std::stof(tokens[0]);
    point.y() = std::stof(tokens[1]);
    point.z() = std::stof(tokens[2]);
    points_.push_back(point);

    // Read normals if available
    if (has_normals_) {
      CHECK_GE(nx_idx_, 0);
      CHECK_GE(ny_idx_, 0);
      CHECK_GE(nz_idx_, 0);
      if (static_cast<size_t>(nx_idx_) < tokens.size() &&
          static_cast<size_t>(ny_idx_) < tokens.size() &&
          static_cast<size_t>(nz_idx_) < tokens.size()) {
        Vector3f normal;
        normal.x() = std::stof(tokens[nx_idx_]);
        normal.y() = std::stof(tokens[ny_idx_]);
        normal.z() = std::stof(tokens[nz_idx_]);
        normals_.push_back(normal);
      } else {
        LOG(ERROR) << "Incomplete normal data at vertex " << i;
        return false;
      }
    }

    // Read intensity if available
    if (has_intensities_) {
      CHECK_GE(intensity_idx_, 0);
      if (static_cast<size_t>(intensity_idx_) < tokens.size()) {
        const float intensity = std::stof(tokens[intensity_idx_]);
        intensities_.push_back(intensity);
      } else {
        LOG(ERROR) << "Incomplete intensity data at vertex " << i;
        return false;
      }
    }

    // Read timestamp if available
    if (has_timestamps_) {
      CHECK_GE(timestamp_idx_, 0);
      if (static_cast<size_t>(timestamp_idx_) < tokens.size()) {
        // Read timestamp in seconds (as written by PlyWriter)
        const float timestamp_seconds = std::stof(tokens[timestamp_idx_]);
        // Convert seconds back to milliseconds
        constexpr float kSecondsToMilliSeconds = 1e3f;
        const int64_t timestamp_ms =
            static_cast<int64_t>(timestamp_seconds * kSecondsToMilliSeconds);
        timestamps_ms_.push_back(Time(timestamp_ms));
      } else {
        LOG(ERROR) << "Incomplete timestamp data at vertex " << i;
        return false;
      }
    }

    // Read colors if available
    if (has_colors_) {
      CHECK_GE(red_idx_, 0);
      CHECK_GE(green_idx_, 0);
      CHECK_GE(blue_idx_, 0);
      if (static_cast<size_t>(red_idx_) < tokens.size() &&
          static_cast<size_t>(green_idx_) < tokens.size() &&
          static_cast<size_t>(blue_idx_) < tokens.size()) {
        Color color;
        color.r() = static_cast<uint8_t>(std::stoi(tokens[red_idx_]));
        color.g() = static_cast<uint8_t>(std::stoi(tokens[green_idx_]));
        color.b() = static_cast<uint8_t>(std::stoi(tokens[blue_idx_]));
        colors_.push_back(color);
      } else {
        LOG(ERROR) << "Incomplete color data at vertex " << i;
        return false;
      }
    }
  }

  // Verify that all data was successfully read
  if (has_normals_ && normals_.size() != points_.size()) {
    LOG(ERROR) << "Normal count (" << normals_.size()
               << ") does not match point count (" << points_.size() << ")";
    return false;
  }

  if (has_intensities_ && intensities_.size() != points_.size()) {
    LOG(ERROR) << "Intensity count (" << intensities_.size()
               << ") does not match point count (" << points_.size() << ")";
    return false;
  }

  if (has_timestamps_ && timestamps_ms_.size() != points_.size()) {
    LOG(ERROR) << "Timestamp count (" << timestamps_ms_.size()
               << ") does not match point count (" << points_.size() << ")";
    return false;
  }

  if (has_colors_ && colors_.size() != points_.size()) {
    LOG(ERROR) << "Color count (" << colors_.size()
               << ") does not match point count (" << points_.size() << ")";
    return false;
  }

  return true;
}

bool PlyParser::readFaces() {
  std::string line;
  constexpr int kTriangleSize = 3;

  for (size_t i = 0; i < num_faces_; i++) {
    if (!std::getline(file_, line)) {
      LOG(ERROR) << "Unexpected end of file at face " << i;
      return false;
    }

    std::istringstream iss(line);
    int vertex_count;
    iss >> vertex_count;

    // Only support triangles (3 vertices per face)
    // This matches what PlyWriter outputs
    if (vertex_count != kTriangleSize) {
      LOG(ERROR) << "Only triangle faces are supported, got face with "
                 << vertex_count << " vertices at face " << i;
      return false;
    }

    // Read the three vertex indices
    for (int j = 0; j < kTriangleSize; j++) {
      int vertex_idx;
      if (!(iss >> vertex_idx)) {
        LOG(ERROR) << "Failed to read vertex index " << j << " for face " << i;
        return false;
      }
      triangles_.push_back(vertex_idx);
    }
  }

  // Verify triangle data
  if (triangles_.size() != num_faces_ * 3) {
    LOG(WARNING) << "Triangle count mismatch. Expected " << num_faces_ * 3
                 << " indices, got " << triangles_.size();
    return false;
  }

  return true;
}

bool PlyParser::toMesh(ColorMesh* mesh, const CudaStream& cuda_stream) const {
  if (!is_valid_) {
    LOG(ERROR)
        << "Cannot convert to mesh: PLY file was not successfully parsed.";
    return false;
  }
  if (mesh == nullptr || points_.empty() || !has_triangles_ ||
      triangles_.empty()) {
    LOG(ERROR) << "Cannot convert ply file to mesh: missing mesh data.";
    return false;
  }

  // Clear the mesh
  mesh->clearNoDeallocate();

  // Copy data using the provided CUDA stream
  mesh->vertices.copyFromAsync(points_, cuda_stream);

  // Copy normals if available
  if (has_normals_ && !normals_.empty()) {
    mesh->vertex_normals.copyFromAsync(normals_, cuda_stream);
  }

  // Copy colors if available
  if (has_colors_ && !colors_.empty()) {
    mesh->vertex_appearances.copyFromAsync(colors_, cuda_stream);
  }

  // Copy triangles
  mesh->triangles.copyFromAsync(triangles_, cuda_stream);

  return true;
}

bool PlyParser::toPointcloud(Pointcloud* pointcloud,
                             const CudaStream& cuda_stream) const {
  if (!is_valid_) {
    LOG(ERROR) << "Cannot convert to pointcloud: PLY file was not successfully "
                  "parsed.";
    return false;
  }
  if (pointcloud == nullptr || points_.empty()) {
    LOG(ERROR) << "Cannot convert ply file to pointcloud: missing point data.";
    return false;
  }

  // Copy points to the pointcloud
  pointcloud->copyPointsFromAsync(points_, cuda_stream);

  // Copy timestamps if available
  if (has_timestamps_) {
    CHECK_EQ(timestamps_ms_.size(), points_.size());
    pointcloud->copyTimestampsFromAsync(timestamps_ms_, cuda_stream);
  }

  return true;
}

}  // namespace io
}  // namespace nvblox
