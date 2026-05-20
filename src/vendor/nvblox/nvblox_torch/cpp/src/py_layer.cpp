/*
 * Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 */

#include "nvblox_torch/py_layer.h"

namespace pynvblox {

nvblox::Index3D toIndex3D(const torch::Tensor& index) {
  CHECK_EQ(index.sizes().size(), 1);
  CHECK_EQ(index.sizes()[0], 3);
  CHECK_EQ(index.dtype(), torch::kInt32);
  auto accessor = index.accessor<int32_t, 1>();
  return nvblox::Index3D(accessor[0], accessor[1], accessor[2]);
}

// Tensor from TSDF block
torch::Tensor tensorFromBlock(nvblox::TsdfBlock* block_ptr) {
  // Wrap
  constexpr int kVoxelsPerSide = nvblox::TsdfBlock::kVoxelsPerSide;
  const auto options =
      torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA, 0);
  constexpr int kNumChannelsTsdfVoxel = 2;
  return torch::from_blob(static_cast<void*>(block_ptr),  // Data
                          {kVoxelsPerSide, kVoxelsPerSide, kVoxelsPerSide,
                           kNumChannelsTsdfVoxel},  // Sizes
                          options);
}

// Tensor from feature block
torch::Tensor tensorFromBlock(nvblox::FeatureBlock* block_ptr) {
  // Wrap
  constexpr int kVoxelsPerSide = nvblox::FeatureBlock::kVoxelsPerSide;
  const auto options =
      torch::TensorOptions().dtype(torch::kFloat16).device(torch::kCUDA, 0);
  return torch::from_blob(static_cast<void*>(block_ptr),  // Data
                          {kVoxelsPerSide, kVoxelsPerSide, kVoxelsPerSide,
                           nvblox::FeatureArray::size() + 1},  // Sizes
                          options);
}

// Tensor from Color block
torch::Tensor tensorFromBlock(nvblox::ColorBlock* block_ptr) {
  // Wrap
  constexpr int kVoxelsPerSide = nvblox::ColorBlock::kVoxelsPerSide;
  const auto options =
      torch::TensorOptions().dtype(torch::kUInt8).device(torch::kCUDA, 0);
  constexpr int kNumChannelsColorVoxel = 3;
  constexpr int kSizeOfColorVoxel = sizeof(nvblox::ColorVoxel);
  return torch::from_blob(
      static_cast<void*>(block_ptr),  // Data
      {kVoxelsPerSide, kVoxelsPerSide, kVoxelsPerSide,
       kNumChannelsColorVoxel},  // Sizes
      // NOTE(alexmillane. 2025-05-12): Color block has non-uniform types. It's
      // 3 uint8_t of RGB, then 1 byte of padding, then 1 float of weight.
      // Currently we don't support returning tensors which wrap non-uniform
      // types. So we're returning a tensor with the RGB values, and striding
      // over the weights.
      {kVoxelsPerSide * kVoxelsPerSide * kSizeOfColorVoxel,
       kVoxelsPerSide * kSizeOfColorVoxel, kSizeOfColorVoxel,
       sizeof(nvblox::Color::value_type)},  // strides
      options);
}

torch::Tensor tensorFromIndex(const nvblox::Index3D& block_idx) {
  const auto options =
      torch::TensorOptions().dtype(torch::kInt32).device(torch::kCPU);
  torch::Tensor tensor = torch::empty({3}, options);
  auto accessor = tensor.accessor<int32_t, 1>();
  accessor[0] = block_idx.x();
  accessor[1] = block_idx.y();
  accessor[2] = block_idx.z();
  return tensor;
}

template <typename NativeLayerType>
PyVoxelBlockLayer<NativeLayerType>::PyVoxelBlockLayer(double voxel_size_m)
    : PyVoxelBlockLayer(voxel_size_m, nvblox::MemoryType::kDevice) {}

template <typename NativeLayerType>
PyVoxelBlockLayer<NativeLayerType>::PyVoxelBlockLayer(
    double voxel_size_m, nvblox::MemoryType memory_type)
    : layer_(std::make_shared<NativeLayerType>(static_cast<float>(voxel_size_m),
                                               memory_type)) {}

template <typename NativeLayerType>
PyVoxelBlockLayer<NativeLayerType>::PyVoxelBlockLayer(
    std::shared_ptr<NativeLayerType> layer)
    : layer_(layer) {}

template <typename NativeLayerType>
int64_t PyVoxelBlockLayer<NativeLayerType>::numBlocks() const {
  return static_cast<int64_t>(layer_->numBlocks());
}

template <typename NativeLayerType>
int64_t PyVoxelBlockLayer<NativeLayerType>::numAllocatedBytes() const {
  return static_cast<int64_t>(layer_->numAllocatedBytes());
}

template <typename NativeLayerType>
int64_t PyVoxelBlockLayer<NativeLayerType>::numAllocatedBlocks() const {
  return static_cast<int64_t>(layer_->numAllocatedBlocks());
}

template <typename NativeLayerType>
bool PyVoxelBlockLayer<NativeLayerType>::isBlockAllocated(
    const torch::Tensor& index) {
  return layer_->isBlockAllocated(toIndex3D(index));
}

template <typename NativeLayerType>
double PyVoxelBlockLayer<NativeLayerType>::voxel_size() const {
  return layer_->voxel_size();
}

template <typename NativeLayerType>
void PyVoxelBlockLayer<NativeLayerType>::clear() {
  layer_->clear();
}

template <typename NativeLayerType>
c10::intrusive_ptr<PyVoxelBlockLayer<NativeLayerType>>
PyVoxelBlockLayer<NativeLayerType>::clone() const {
  auto layer_ptr = c10::make_intrusive<PyVoxelBlockLayer>(
      layer_->voxel_size(), layer_->memory_type());
  layer_ptr->layer_->copyFrom(*layer_);
  return layer_ptr;
}

template <typename NativeLayerType>
void PyVoxelBlockLayer<NativeLayerType>::allocateBlockAtIndex(
    const torch::Tensor& index) {
  layer_->allocateBlockAtIndex(toIndex3D(index));
}

template <typename NativeLayerType>
torch::Tensor PyVoxelBlockLayer<NativeLayerType>::getBlockAtIndex(
    const torch::Tensor& index) {
  auto block_ptr = layer_->getBlockAtIndex(toIndex3D(index));
  if (!block_ptr) {
    return torch::Tensor();
  }
  return tensorFromBlock(block_ptr.get());
}

template <typename NativeLayerType>
torch::Tensor PyVoxelBlockLayer<NativeLayerType>::getAllBlockIndices() {
  if (layer_->numBlocks() == 0) {
    return torch::Tensor();
  }
  // Allocate Tensor
  auto options =
      torch::TensorOptions().dtype(torch::kInt32).device(torch::kCPU);
  const int height = layer_->numBlocks();
  constexpr int kIndicesWidth = 3;
  torch::Tensor indices = torch::zeros({height, kIndicesWidth}, options);
  // Populate tensor
  auto accessor = indices.accessor<int32_t, 2>();
  const std::vector<nvblox::Index3D> block_indices =
      layer_->getAllBlockIndices();
  for (int row_idx = 0; row_idx < height; row_idx++) {
    const nvblox::Index3D& block_idx = block_indices[row_idx];
    accessor[row_idx][0] = block_idx.x();
    accessor[row_idx][1] = block_idx.y();
    accessor[row_idx][2] = block_idx.z();
  }
  return indices;
}
template <typename NativeLayerType>
std::tuple<std::vector<torch::Tensor>, std::vector<torch::Tensor>>
PyVoxelBlockLayer<NativeLayerType>::getAllBlocks() {
  // Lists of blocks and their indices
  std::vector<torch::Tensor> block_tensors;
  std::vector<torch::Tensor> block_index_tensors;
  block_tensors.reserve(layer_->numBlocks());
  block_index_tensors.reserve(layer_->numBlocks());
  // Add each block to the lists
  const std::vector<nvblox::Index3D> block_indices =
      layer_->getAllBlockIndices();
  for (const nvblox::Index3D& block_idx : block_indices) {
    // Get the block
    auto block_ptr = layer_->getBlockAtIndex(block_idx);
    assert(block_ptr);
    // Wrap the block.
    block_tensors.push_back(tensorFromBlock(block_ptr.get()));
    // Add its index.
    block_index_tensors.push_back(tensorFromIndex(block_idx));
  }
  return {block_tensors, block_index_tensors};
}

// Specializations
template class PyVoxelBlockLayer<nvblox::TsdfLayer>;
template class PyVoxelBlockLayer<nvblox::FeatureLayer>;
template class PyVoxelBlockLayer<nvblox::ColorLayer>;
}  // namespace pynvblox
