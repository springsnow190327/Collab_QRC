/*
 * Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 */
#include "nvblox_torch/py_rendering.h"

#include <nvblox/map/common_names.h>
#include <nvblox/rays/sphere_tracer.h>

#include "nvblox_torch/convert_tensors.h"

namespace pynvblox {

torch::Tensor renderDepthImage(c10::intrusive_ptr<PyTsdfLayer> layer,
                               torch::Tensor camera_pose,
                               torch::Tensor intrinsics, int64_t img_height,
                               int64_t img_width, double max_ray_length,
                               int64_t max_steps) {
  const nvblox::TsdfLayer& tsdf_layer = *layer->layer_;
  // TODO: This 4.0 is the default truncation distance in
  // projective_integrator_base.h This should be made a global constant and
  // somehow set accordingly.
  const float truncation_distance_m = tsdf_layer.voxel_size() * 4.0;

  const nvblox::Transform T_S_C = copy_transform_from_tensor(camera_pose);
  const nvblox::Camera camera =
      camera_from_intrinsics_tensor(intrinsics, img_height, img_width);

  nvblox::SphereTracer sphere_tracer_gpu;
  sphere_tracer_gpu.maximum_ray_length_m(max_ray_length);
  sphere_tracer_gpu.maximum_steps(max_steps);

  torch::Tensor depth_image_t =
      init_depth_image_tensor(img_height, img_width, torch::kCUDA);
  nvblox::DepthImageView depth_image_view =
      view_from_tensor<float>(depth_image_t);
  sphere_tracer_gpu.renderImageOnGPU(camera, T_S_C, tsdf_layer,
                                     truncation_distance_m, &depth_image_view,
                                     nvblox::MemoryType::kDevice);

  return depth_image_t;
}

std::vector<torch::Tensor> renderDepthAndColorImage(
    c10::intrusive_ptr<PyTsdfLayer> py_tsdf_layer,
    c10::intrusive_ptr<PyColorLayer> py_color_layer, torch::Tensor camera_pose,
    torch::Tensor intrinsics, int64_t img_height, int64_t img_width,
    double max_ray_length, int64_t max_steps) {
  const nvblox::TsdfLayer& tsdf_layer = *py_tsdf_layer->layer_;
  const nvblox::ColorLayer& color_layer = *py_color_layer->layer_;
  // TODO: This 4.0 is the default truncation distance in
  // projective_integrator_base.h This should be made a global constant and
  // somehow set accordingly.
  CHECK_EQ(tsdf_layer.voxel_size(), color_layer.voxel_size());
  double truncation_distance_m = tsdf_layer.voxel_size() * 4.0;

  const nvblox::Transform T_S_C = copy_transform_from_tensor(camera_pose);
  const nvblox::Camera camera =
      camera_from_intrinsics_tensor(intrinsics, img_height, img_width);

  nvblox::SphereTracer sphere_tracer_gpu;
  sphere_tracer_gpu.maximum_ray_length_m(max_ray_length);
  sphere_tracer_gpu.maximum_steps(max_steps);

  torch::Tensor depth_image_t =
      init_depth_image_tensor(img_height, img_width, torch::kCUDA);
  nvblox::DepthImageView depth_image_view =
      view_from_tensor<float>(depth_image_t);
  torch::Tensor color_image_t =
      init_color_image_tensor(img_height, img_width, torch::kCUDA);
  nvblox::ColorImageView color_image_view =
      view_from_tensor<nvblox::Color>(color_image_t);

  sphere_tracer_gpu.renderRgbdImageOnGPU(
      camera, T_S_C, tsdf_layer, color_layer, truncation_distance_m,
      &depth_image_view, &color_image_view, nvblox::MemoryType::kDevice);

  return {depth_image_t, color_image_t};
}

}  // namespace pynvblox
