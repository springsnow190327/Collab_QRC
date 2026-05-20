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
#include "nvblox/map/blox.h"
#include "nvblox/map/common_names.h"

namespace nvblox {

// NOTE(alexmillane): This union is used to zero the padding byte in the
// ColorVoxel.
//                    See below for more details.
union ColorVoxelUnion {
  ColorVoxel voxel;
  uint8_t raw[sizeof(ColorVoxel)];
};

// Must be called with:
// - a single block
// - one thread per voxel
__global__ void setColorBlockGray(ColorBlock* block_device_ptr) {
  ColorVoxel* voxel_ptr =
      &block_device_ptr->voxels[threadIdx.z][threadIdx.y][threadIdx.x];
  // NOTE(alexmillane, 2025.05.02): This code below is being used to
  // a) Zero all bytes in the ColorVoxelUnion (including the padding byte),
  // b) Set the color to gray, and
  // c) set the weight to 0.0f.
  // We're taking care to zero the padding byte because the compute-sanitizer
  // initcheck was firing when copying the ColorVoxel struct because when
  // initializing naively, because the padding byte was uninitialized.
  ColorVoxelUnion union_voxel = {};
  union_voxel.voxel.color = Color::Gray();
  union_voxel.voxel.weight = 0.0f;
  memcpy(voxel_ptr, &union_voxel, sizeof(ColorVoxelUnion));
}

void setColorBlockGrayOnGPUAsync(ColorBlock* block_device_ptr,
                                 const CudaStream& cuda_stream) {
  constexpr int kVoxelsPerSide = VoxelBlock<bool>::kVoxelsPerSide;
  const dim3 kThreadsPerBlock(kVoxelsPerSide, kVoxelsPerSide, kVoxelsPerSide);
  setColorBlockGray<<<1, kThreadsPerBlock, 0, cuda_stream>>>(block_device_ptr);
  checkCudaErrors(cudaPeekAtLastError());
}

template <class BlockType>
__global__ void initializeBlocksKernel(BlockType** block_ptrs, int num_blocks) {
  const int block_idx = blockIdx.x;

  if (block_idx < num_blocks) {
    block_ptrs[block_idx]->voxels[threadIdx.z][threadIdx.y][threadIdx.x] =
        BlockType::VoxelType();
  }
}

template <class BlockType>
void initializeBlocksAsync(host_vector<BlockType*>& blocks,
                           const CudaStream& cuda_stream,
                           const MemoryType /*unused*/) {
  if (blocks.empty()) {
    return;
  }

  const dim3 threads_per_block = {BlockType::kVoxelsPerSide,
                                  BlockType::kVoxelsPerSide,
                                  BlockType::kVoxelsPerSide};
  const int num_blocks = blocks.size();

  initializeBlocksKernel<BlockType>
      <<<num_blocks, threads_per_block, 0, cuda_stream>>>(blocks.data(),
                                                          blocks.size());
}

// Specialization for ColorMeshBlock
template <>
void initializeBlocksAsync(host_vector<ColorMeshBlock*>& blocks,
                           const CudaStream& cuda_stream,
                           const MemoryType memory_type) {
  for (auto& ptr : blocks) {
    ColorMeshBlock::initAsync(ptr, memory_type, cuda_stream);
  }
}

// Specialization for FeatureMeshBlock
template <>
void initializeBlocksAsync(host_vector<FeatureMeshBlock*>& blocks,
                           const CudaStream& cuda_stream,
                           const MemoryType memory_type) {
  for (auto& ptr : blocks) {
    FeatureMeshBlock::initAsync(ptr, memory_type, cuda_stream);
  }
}

// Specializations for Voxelblock types
template void initializeBlocksAsync<TsdfBlock>(host_vector<TsdfBlock*>& blocks,
                                               const CudaStream& cuda_stream,
                                               const MemoryType memory_type);
template void initializeBlocksAsync<OccupancyBlock>(
    host_vector<OccupancyBlock*>& blocks, const CudaStream& cuda_stream,
    const MemoryType memory_type);
template void initializeBlocksAsync<ColorBlock>(
    host_vector<ColorBlock*>& blocks, const CudaStream& cuda_stream,
    const MemoryType memory_type);
template void initializeBlocksAsync<FeatureBlock>(
    host_vector<FeatureBlock*>& blocks, const CudaStream& cuda_stream,
    const MemoryType memory_type);
template void initializeBlocksAsync<ColorMeshBlock>(
    host_vector<ColorMeshBlock*>& blocks, const CudaStream& cuda_stream,
    const MemoryType memory_type);
template void initializeBlocksAsync<FeatureMeshBlock>(
    host_vector<FeatureMeshBlock*>& blocks, const CudaStream& cuda_stream,
    const MemoryType memory_type);
template void initializeBlocksAsync<FreespaceBlock>(
    host_vector<FreespaceBlock*>& blocks, const CudaStream& cuda_stream,
    const MemoryType memory_type);
template void initializeBlocksAsync<EsdfBlock>(host_vector<EsdfBlock*>& blocks,
                                               const CudaStream& cuda_stream,
                                               const MemoryType memory_type);

}  // namespace nvblox
