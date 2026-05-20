/*
 * Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 */
#pragma once

#include <memory>

#include <torch/script.h>

#include <ATen/ATen.h>
#include <torch/custom_class.h>  // This is the file that contains info about torch+class

#include <nvblox/map/common_names.h>

namespace pynvblox {

torch::Tensor tensorFromBlock(nvblox::TsdfBlock* block_ptr);
torch::Tensor tensorFromIndex(const nvblox::Index3D& index);

template <typename _NativeLayerType>
struct PyVoxelBlockLayer : torch::CustomClassHolder {
  using NativeLayerType = _NativeLayerType;

  // Constructor
  PyVoxelBlockLayer(double voxel_size_m);
  PyVoxelBlockLayer(double voxel_size_m, nvblox::MemoryType memory_type);
  PyVoxelBlockLayer(std::shared_ptr<NativeLayerType> layer);

  double voxel_size() const;

  int64_t numBlocks() const;

  int64_t numAllocatedBytes() const;

  int64_t numAllocatedBlocks() const;

  void clear();

  void allocateBlockAtIndex(const torch::Tensor& index);

  torch::Tensor getBlockAtIndex(const torch::Tensor& index);

  bool isBlockAllocated(const torch::Tensor& index);

  torch::Tensor getAllBlockIndices();

  std::tuple<std::vector<torch::Tensor>, std::vector<torch::Tensor>>
  getAllBlocks();

  c10::intrusive_ptr<PyVoxelBlockLayer> clone() const;

  std::shared_ptr<NativeLayerType> layer_;
};

using PyTsdfLayer = PyVoxelBlockLayer<nvblox::TsdfLayer>;
using PyFeatureLayer = PyVoxelBlockLayer<nvblox::FeatureLayer>;
using PyColorLayer = PyVoxelBlockLayer<nvblox::ColorLayer>;

}  // namespace pynvblox
