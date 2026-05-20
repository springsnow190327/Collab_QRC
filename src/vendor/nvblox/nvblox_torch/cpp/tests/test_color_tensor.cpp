/*
 * Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <nvblox/map/common_names.h>

#include "nvblox_torch/py_layer.h"

TEST(ColorTensor, WrapLayer) {
  // Create a color layer.
  constexpr float kVoxelSize = 0.1;
  auto color_layer = std::make_shared<nvblox::ColorLayer>(
      kVoxelSize, nvblox::MemoryType::kUnified);
  auto color_block =
      color_layer->allocateBlockAtIndex(nvblox::Index3D(0, 0, 0));

  // Set all voxels to red.
  for (int x = 0; x < nvblox::ColorBlock::kVoxelsPerSide; ++x) {
    for (int y = 0; y < nvblox::ColorBlock::kVoxelsPerSide; ++y) {
      for (int z = 0; z < nvblox::ColorBlock::kVoxelsPerSide; ++z) {
        color_block->voxels[x][y][z].color = nvblox::Color(1, 2, 3);
      }
    }
  }

  // Wrap
  auto color_layer_wrapper = pynvblox::PyColorLayer(color_layer);

  // Get the wrapped block
  const auto options =
      torch::TensorOptions().dtype(torch::kInt32).device(torch::kCPU);
  auto index = torch::tensor({0, 0, 0}, options);
  torch::Tensor block_tensor = color_layer_wrapper.getBlockAtIndex(index);

  // Loop over block tensor and verify values
  auto accessor = block_tensor.accessor<uint8_t, 4>();
  for (int x = 0; x < nvblox::ColorBlock::kVoxelsPerSide; ++x) {
    for (int y = 0; y < nvblox::ColorBlock::kVoxelsPerSide; ++y) {
      for (int z = 0; z < nvblox::ColorBlock::kVoxelsPerSide; ++z) {
        EXPECT_EQ(accessor[x][y][z][0], 1);
        EXPECT_EQ(accessor[x][y][z][1], 2);
        EXPECT_EQ(accessor[x][y][z][2], 3);
      }
    }
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
