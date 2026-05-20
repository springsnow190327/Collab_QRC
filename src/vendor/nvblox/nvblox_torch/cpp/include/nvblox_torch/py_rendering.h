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

#pragma once

#include <torch/script.h>

#include <ATen/ATen.h>

#include "nvblox_torch/py_layer.h"

namespace pynvblox {

torch::Tensor renderDepthImage(c10::intrusive_ptr<PyTsdfLayer> layer,
                               torch::Tensor camera_pose,
                               torch::Tensor intrinsics, int64_t img_height,
                               int64_t img_width, double max_ray_length,
                               int64_t max_steps);

std::vector<torch::Tensor> renderDepthAndColorImage(
    c10::intrusive_ptr<PyTsdfLayer> py_tsdf_layer,
    c10::intrusive_ptr<PyColorLayer> py_color_layer, torch::Tensor camera_pose,
    torch::Tensor intrinsics, int64_t img_height, int64_t img_width,
    double max_ray_length, int64_t max_steps);

}  // namespace pynvblox
