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
#include <gtest/gtest.h>

#include "nvblox/map/common_names.h"
#include "nvblox/map/internal/block_memory_pool.h"

using namespace nvblox;

TEST(BlockMemoryPool, popBeyondCapacity) {
  // The pool should resize internally if we pop beyond capacity
  static constexpr size_t kNumPreallocatedBlocks = 10;
  BlockMemoryPoolParams params;
  params.num_preallocated_blocks = kNumPreallocatedBlocks;

  BlockMemoryPool<TsdfBlock> pool(params);
  for (size_t i = 0; i < 2 * kNumPreallocatedBlocks; ++i) {
    auto block = pool.popBlock(CudaStreamOwning());
    ASSERT_TRUE(block != nullptr);
  }
}

TEST(BlockMemoryPool, pushAndPop) {
  BlockMemoryPool<TsdfBlock> pool;

  auto popped_block = pool.popBlock(CudaStreamOwning());
  ASSERT_TRUE(popped_block != nullptr);
  pool.pushBlock(popped_block);
  auto repopped_block = pool.popBlock(CudaStreamOwning());

  // We should get the same pointer back
  ASSERT_EQ(popped_block.get(), repopped_block.get());
}

TEST(BlockMemoryPool, numPreallocatedBlocks) {
  BlockMemoryPoolParams params;
  params.num_preallocated_blocks = 3;
  BlockMemoryPool<TsdfBlock> pool(params);
  ASSERT_EQ(pool.numAllocatedBlocks(), params.num_preallocated_blocks);
}

TEST(BlockMemoryPool, expansionFactor) {
  BlockMemoryPoolParams params;
  params.num_preallocated_blocks = 3;
  params.expansion_factor = 11.f;

  BlockMemoryPool<TsdfBlock> pool(params);

  // Pop blocks to trigger resize
  for (int i = 0; i < params.num_preallocated_blocks + 1; ++i) {
    pool.popBlock(CudaStreamOwning());
  }

  ASSERT_EQ(pool.numAllocatedBlocks(),
            params.num_preallocated_blocks * params.expansion_factor);
}

TEST(BlockMemoryPool, expansionFactorZero) {
  BlockMemoryPoolParams params;
  params.num_preallocated_blocks = 0;
  params.expansion_factor = 0.0f;

  BlockMemoryPool<TsdfBlock> pool(params);

  // Num allocated blocks should grow with one every time we pop a block
  constexpr int kNumIterations = 1024;
  for (int i = 0; i < kNumIterations; ++i) {
    ASSERT_EQ(pool.numAllocatedBlocks(), i);
    pool.popBlock(CudaStreamOwning());
  }
}

TEST(BlockMemoryPool, numAllocatedBytes) {
  constexpr int kNumPreallocatedBlocks = 0;
  BlockMemoryPoolParams params;
  params.memory_type = MemoryType::kHost;
  params.num_preallocated_blocks = kNumPreallocatedBlocks;

  BlockMemoryPool<TsdfBlock> pool(params);

  constexpr int kNumIterations = 1024;
  for (int i = 0; i < kNumIterations; ++i) {
    CHECK_EQ(static_cast<size_t>(pool.numAllocatedBytes()),
             pool.numAllocatedBlocks() * sizeof(TsdfBlock));
    pool.popBlock(CudaStreamOwning());
  }
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
