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
#include "nvblox/mesh/mesh_block.h"

namespace nvblox {

template <typename AppearanceType>
MeshBlock<AppearanceType>::MeshBlock(MemoryType memory_type)
    : vertices(memory_type),
      vertex_normals(memory_type),
      vertex_appearances(memory_type),
      triangles(memory_type) {}

template <typename AppearanceType>
void MeshBlock<AppearanceType>::clear() {
  vertices.clearNoDeallocate();
  vertex_normals.clearNoDeallocate();
  triangles.clearNoDeallocate();
  vertex_appearances.clearNoDeallocate();
}

template <typename AppearanceType>
MeshBlock<AppearanceType>::Ptr MeshBlock<AppearanceType>::allocate(
    MemoryType memory_type) {
  return std::make_shared<MeshBlock<AppearanceType>>(memory_type);
}

template <typename AppearanceType>
MeshBlock<AppearanceType>::Ptr MeshBlock<AppearanceType>::allocateAsync(
    MemoryType memory_type, const CudaStream&) {
  return allocate(memory_type);
}

template <typename AppearanceType>
size_t MeshBlock<AppearanceType>::size() const {
  return vertices.size();
}

template <typename AppearanceType>
size_t MeshBlock<AppearanceType>::capacity() const {
  return vertices.capacity();
}

template <typename AppearanceType>
void MeshBlock<AppearanceType>::expandAppearanceToMatchVerticesAsync(
    const CudaStream& cuda_stream) {
  vertex_appearances.reserveAsync(vertices.capacity(), cuda_stream);
  vertex_appearances.resizeAsync(vertices.size(), cuda_stream);
}

template <typename AppearanceType>
void MeshBlock<AppearanceType>::copyFromAsync(
    const MeshBlock<AppearanceType>& other, const CudaStream& cuda_stream) {
  vertices.copyFromAsync(other.vertices, cuda_stream);
  vertex_normals.copyFromAsync(other.vertex_normals, cuda_stream);
  vertex_appearances.copyFromAsync(other.vertex_appearances, cuda_stream);
  triangles.copyFromAsync(other.triangles, cuda_stream);
}

template <typename AppearanceType>
void MeshBlock<AppearanceType>::copyFrom(
    const MeshBlock<AppearanceType>& other) {
  copyFromAsync(other, CudaStreamOwning());
}

// Set the pointers to point to the mesh block.
template <typename AppearanceType>
CudaMeshBlock<AppearanceType>::CudaMeshBlock(MeshBlock<AppearanceType>* block) {
  CHECK_NOTNULL(block);
  vertices = block->vertices.data();
  vertex_normals = block->vertex_normals.data();
  triangles = block->triangles.data();
  vertex_appearances = block->vertex_appearances.data();

  vertices_size = block->vertices.size();
  triangles_size = block->triangles.size();
}

template class MeshBlock<Color>;
template class MeshBlock<FeatureArray>;
template class CudaMeshBlock<Color>;
template class CudaMeshBlock<FeatureArray>;

}  // namespace nvblox
