/*
 * Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */
#include "nvblox_torch/convert_tensors.h"

#include <cuda_runtime.h>

#include "nvblox/core/color.h"
#include "nvblox/core/feature_array.h"

namespace pynvblox {

torch::Tensor init_depth_image_tensor(int64_t height, int64_t width,
                                      torch::DeviceType device) {
  auto options =
      torch::TensorOptions().dtype(torch::kFloat32).device(device, 0);
  torch::Tensor depth_image_t = torch::zeros({height, width}, options);
  return depth_image_t;
}

torch::Tensor init_color_image_tensor(int64_t height, int64_t width,
                                      torch::DeviceType device) {
  auto options = torch::TensorOptions().dtype(torch::kUInt8).device(device, 0);
  torch::Tensor color_image_t =
      torch::zeros({height, width, nvblox::kRgbNumElements}, options);
  return color_image_t;
}

nvblox::DepthImage copy_depth_image_from_tensor(torch::Tensor depth_image_t) {
  CHECK_EQ(depth_image_t.sizes().size(), 2);
  nvblox::MemoryType memory_type =
      memory_type_from_torch_device(depth_image_t.device().type());
  int height = depth_image_t.sizes()[0];
  int width = depth_image_t.sizes()[1];

  nvblox::DepthImage depth_image(memory_type);
  depth_image.copyFrom(height, width, (float*)depth_image_t.data_ptr());
  return depth_image;
}

nvblox::MonoImage copy_mono_image_from_tensor(torch::Tensor mono_image_t) {
  CHECK_EQ(mono_image_t.sizes().size(), 2);
  nvblox::MemoryType memory_type =
      memory_type_from_torch_device(mono_image_t.device().type());
  int height = mono_image_t.sizes()[0];
  int width = mono_image_t.sizes()[1];

  nvblox::MonoImage mono_image(memory_type);
  mono_image.copyFrom(height, width, (uint8_t*)mono_image_t.data_ptr());
  return mono_image;
}

nvblox::ColorImage copy_color_image_from_tensor(torch::Tensor color_image_t) {
  CHECK_EQ(color_image_t.sizes().size(), 3);
  nvblox::MemoryType memory_type =
      memory_type_from_torch_device(color_image_t.device().type());
  int height = color_image_t.sizes()[0];
  int width = color_image_t.sizes()[1];
  CHECK_EQ(color_image_t.sizes()[2], 4);

  // Color is a class with just 3 members, each a uint8. So it should map over
  // the last axis of the tensor
  nvblox::ColorImage color_image(memory_type);
  color_image.copyFrom(height, width, (nvblox::Color*)color_image_t.data_ptr());
  return color_image;
}

nvblox::FeatureImage copy_feature_image_from_tensor(
    torch::Tensor feature_image_t) {
  CHECK_EQ(feature_image_t.sizes().size(), 3);
  nvblox::MemoryType memory_type =
      memory_type_from_torch_device(feature_image_t.device().type());
  int height = feature_image_t.sizes()[0];
  int width = feature_image_t.sizes()[1];
  CHECK_EQ(static_cast<size_t>(feature_image_t.sizes()[2]),
           nvblox::FeatureArray::size());

  nvblox::FeatureImage feature_image(memory_type);
  feature_image.copyFrom(height, width,
                         (nvblox::FeatureArray*)feature_image_t.data_ptr());
  return feature_image;
}

torch::Tensor copy_depth_image_to_tensor(
    const nvblox::DepthImage& depth_image) {
  torch::DeviceType device =
      memory_type_to_torch_device(depth_image.memory_type());
  int height = depth_image.height();
  int width = depth_image.width();

  torch::Tensor depth_image_t = init_depth_image_tensor(height, width, device);

  depth_image.copyTo(depth_image_t.data_ptr<float>());

  return depth_image_t;
}

torch::Tensor copy_color_image_to_tensor(
    const nvblox::ColorImage& color_image) {
  torch::DeviceType device =
      memory_type_to_torch_device(color_image.memory_type());
  int height = color_image.height();
  int width = color_image.width();

  torch::Tensor color_image_t = init_color_image_tensor(height, width, device);

  color_image.copyTo(static_cast<nvblox::Color*>(color_image_t.data_ptr()));

  return color_image_t;
}

nvblox::Transform copy_transform_from_tensor(torch::Tensor transform_t) {
  // Torch uses row-major, while Eigen uses column-major matrices. Transpose to
  // convert:
  torch::Tensor transform_t_T = transform_t.transpose(0, 1).contiguous();

  nvblox::Transform transform;

  cudaMemcpy(transform.matrix().data(), transform_t_T.data_ptr(),
             4 * 4 * sizeof(float), cudaMemcpyDefault);
  return transform;
}

nvblox::Camera camera_from_intrinsics_tensor(torch::Tensor intrinsics_t,
                                             int height, int width) {
  // Convert intrinsics tensor to nvblox Camera
  // make a tensor accessor
  auto intr_a = intrinsics_t.accessor<float, 2>();
  float fu = (float)intr_a[0][0];
  float fv = (float)intr_a[1][1];
  float cu = (float)intr_a[0][2];
  float cv = (float)intr_a[1][2];
  nvblox::Camera camera(fu, fv, cu, cv, width, height);
  return camera;
}

nvblox::MemoryType memory_type_from_torch_device(
    torch::DeviceType device_type) {
  // TODO: Figure out what happens on Jetsons with unified memory, how do we
  // detect kUnified?
  nvblox::MemoryType memory_type =
      (device_type == torch::kCUDA ? nvblox::MemoryType::kDevice
                                   : nvblox::MemoryType::kHost);
  return memory_type;
}

torch::DeviceType memory_type_to_torch_device(nvblox::MemoryType memory_type) {
  // TODO: Figure out what happens on Jetsons with unified memory, how do we
  // assign kUnified?
  torch::DeviceType device_type =
      (memory_type == nvblox::MemoryType::kHost ? torch::kCPU : torch::kCUDA);
  return device_type;
}

}  // namespace pynvblox
