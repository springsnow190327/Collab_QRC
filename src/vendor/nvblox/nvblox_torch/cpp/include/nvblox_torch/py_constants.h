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
#pragma once

#include "nvblox/core/feature_array.h"
#include "nvblox_torch/sdf_query.cuh"

namespace pynvblox {

struct Constants : torch::CustomClassHolder {
  constexpr int64_t featureArrayNumElements() {
    return nvblox::FeatureArray::size();
  }
  constexpr int64_t featureArrayElementSize() {
    return sizeof(nvblox::FeatureArray::value_type);
  }
  constexpr double kESDFUnknownDistance() {
    return static_cast<double>(sdf::kESDFUnknownDistance);
  }
};
}  // namespace pynvblox
