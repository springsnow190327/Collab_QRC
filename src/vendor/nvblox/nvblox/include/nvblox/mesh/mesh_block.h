/*
Copyright 2022 NVIDIA CORPORATION

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

#include <memory>

#include "nvblox/core/color.h"
#include "nvblox/core/cuda_stream.h"
#include "nvblox/core/types.h"
#include "nvblox/core/unified_ptr.h"
#include "nvblox/core/unified_vector.h"
#include "nvblox/map/blox.h"

namespace nvblox {

/// A mesh block containing all of the triangles from this block.
/// Each block contains only the UPPER part of its neighbors: i.e., the max
/// x, y, and z axes. Its neighbors are responsible for the rest.
template <typename AppearanceType>
struct MeshBlock {
  typedef std::shared_ptr<MeshBlock> Ptr;
  typedef std::shared_ptr<const MeshBlock> ConstPtr;

  /// Create a mesh block of the specified memory type.
  MeshBlock(MemoryType memory_type = MemoryType::kDevice);

  void copyFromAsync(const MeshBlock& other, const CudaStream& cuda_stream);
  void copyFrom(const MeshBlock& other);

  // Mesh Data
  // These unified vectors contain the mesh data for this block. Note that
  // Colors and/or intensities are optional. The "triangles" vector is a
  // vector of indices into the vertices vector. Triplets of consecutive
  // elements form triangles with the indexed vertices as their corners.
  unified_vector<Vector3f> vertices;
  unified_vector<Vector3f> vertex_normals;
  unified_vector<AppearanceType> vertex_appearances;
  unified_vector<int> triangles;

  /// Clear all data within the mesh block.
  void clear();

  /// Size of the vertices vector.
  size_t size() const;
  /// Capacity (allocated size) of the vertices vector.
  size_t capacity() const;

  /// Resize colors/intensities such that:
  /// `colors.size()/intensities.size() == vertices.size()`
  void expandAppearanceToMatchVerticesAsync(const CudaStream& cuda_stream);

  /// Note(alexmillane): Memory type ignored, MeshBlocks live in CPU memory.
  static Ptr allocate(MemoryType memory_type);

  /// Note(dtingdahl): Required to comply with common block interface. Cuda
  /// stream is not used, MeshBlocks live in CPU memory.
  static Ptr allocateAsync(MemoryType memory_type, const CudaStream&);

  /// Note(dtingdahl): Required to comply with common block interface. does
  /// nothing.
  static void initAsync(MeshBlock*, const MemoryType, const CudaStream&) {}
};

template <typename AppearanceType>
size_t sizeInBytes(const MeshBlock<AppearanceType>* mesh_block) {
  return mesh_block->vertices.size() * sizeof(Vector3f) +        // NOLINT
         mesh_block->vertex_normals.size() * sizeof(Vector3f) +  // NOLINT
         mesh_block->vertex_appearances.size() *
             sizeof(AppearanceType) +  // NOLINT
         mesh_block->triangles.size() * sizeof(int);
}

/// Helper struct for mesh blocks on CUDA.
/// NOTE: We need this because we can't pass MeshBlock to kernel functions
/// because of the presence of unified_vector members.
template <typename AppearanceType>
struct CudaMeshBlock {
  CudaMeshBlock() = default;
  CudaMeshBlock(MeshBlock<AppearanceType>* block);

  Vector3f* vertices;
  Vector3f* vertex_normals;
  int* triangles;
  AppearanceType* vertex_appearances;
  int vertices_size = 0;
  int triangles_size = 0;
};

}  // namespace nvblox
