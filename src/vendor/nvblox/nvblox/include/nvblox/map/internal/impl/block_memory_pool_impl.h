/*
Copyright 2023 NVIDIA CORPORATION

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

#include "nvblox/map/internal/block_memory_pool.h"

namespace nvblox {

template <class BlockType>
BlockMemoryPool<BlockType>::BlockMemoryPool(const BlockMemoryPoolParams params)
    : params_(params) {
  if (params_.num_preallocated_blocks > 0) {
    expand(params_.num_preallocated_blocks, CudaStreamOwning());
  }
}

template <class BlockType>
typename BlockType::Ptr BlockMemoryPool<BlockType>::popBlock(
    const CudaStream& cuda_stream) {
  // Zero blocks that are being re-used
  if (!recycled_blocks_.empty()) {
    initializeBlocksAsync<BlockType>(recycled_blocks_, cuda_stream,
                                     params_.memory_type);
    recycled_blocks_.clearNoDeallocate();
    cuda_stream.synchronize();
  }

  // Expand if needed.
  if (blocks_.size() == 0) {
    expand(std::max(1, static_cast<int>((params_.expansion_factor - 1) *
                                        num_allocated_blocks_)),
           cuda_stream);
  }

  // Return a ready-to-use block
  typename BlockType::Ptr popped = blocks_.top();
  blocks_.pop();
  return popped;
}

template <class BlockType>
void BlockMemoryPool<BlockType>::pushBlock(typename BlockType::Ptr block) {
  recycled_blocks_.push_back(block.get());
  blocks_.push(block);
}

template <class BlockType>
void BlockMemoryPool<BlockType>::expand(const size_t num_blocks_to_allocate,
                                        const CudaStream& cuda_stream) {
  for (size_t i = 0; i < num_blocks_to_allocate; ++i) {
    blocks_.push(BlockType::allocateAsync(params_.memory_type, cuda_stream));
  }
  num_allocated_blocks_ += num_blocks_to_allocate;

  VLOG(5) << "Expanding the memory pool with " << num_blocks_to_allocate
          << " blocks. Number of allocated blocks: " << num_allocated_blocks_
          << ". Size in mb: " << (numAllocatedBytes() >> 20);

  cuda_stream.synchronize();
}
}  // namespace nvblox
