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
#include <vector>

#include <torch/script.h>

#include <ATen/ATen.h>
#include <torch/custom_class.h>

#include <nvblox/core/types.h>
#include <nvblox/map/common_names.h>
#include <nvblox/mesh/mesh.h>
#include "nvblox/serialization/mesh_serializer_gpu.h"

namespace pynvblox {

template <typename NativeAppearanceType>
struct PyMesh : torch::CustomClassHolder {
  using NativeMeshType = nvblox::SerializedMeshLayer<NativeAppearanceType>;
  using NativeMeshPtr = std::shared_ptr<NativeMeshType>;

  // Constructor
  PyMesh(NativeMeshPtr mesh = std::make_shared<NativeMeshType>())
      : mesh_(mesh) {}

  // Getters returning tensor views of the mesh
  torch::Tensor vertices() const;
  torch::Tensor triangles() const;
  torch::Tensor vertex_appearances() const;

  NativeMeshPtr mesh_;
};

using PyColorMesh = PyMesh<nvblox::Color>;
using PyFeatureMesh = PyMesh<nvblox::FeatureArray>;

}  // namespace pynvblox
