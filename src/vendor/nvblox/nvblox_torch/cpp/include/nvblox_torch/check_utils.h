/*
 * Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */
#pragma once

#include <torch/script.h>

namespace pynvblox {

/// @brief Checks that all tensors are on the GPU.
/// @tparam ...Args A variable list of tensors.
/// @param ...args Tensors.
/// @return True if all tensors are on the GPU.
template <typename... Args>
inline bool checkAllOnGPU(const Args&... args);

/// @brief Checks that the tensor has the specifies sizes.
/// @tparam N The dimensionality of tensor.
/// @param tensor The tensor to check.
/// @param sizes A list of sizes. One size per dimension.
/// @return True if the tensor has the specified sizes.
template <size_t N>
inline bool checkSizes(const torch::Tensor& tensor, const int (&sizes)[N]);

/// @brief Checks that the two input tensors have the same sizes.
/// @param tensor_1 The first tensor.
/// @param tensor_2 The second tensor.
/// @return True if the tensors have the same sizes.
inline bool checkSizesEqual(const torch::Tensor& tensor_1,
                            const torch::Tensor& tensor_2);

/// @brief Checks that the two input images have the same height as one another,
///        and the same width as one another.
///        This is useful for checking that the height and width of an images
///        are the same, even if they have different channel lengths.
/// @param tensor_1 The first tensor.
/// @param tensor_2 The second tensor.
/// @return True if the tensors have the same height and width.
inline bool checkImageDimensionsEqual(const torch::Tensor& tensor_1,
                                      const torch::Tensor& tensor_2);

/// Checks that all tensors are on the GPU or returns.
/// Logs a warning if all the tensors are not on the GPU.
#define ALL_ON_GPU_OR_RETURN(...)                         \
  if (!checkAllOnGPU(__VA_ARGS__)) {                      \
    LOG(WARNING) << "Inputs: " << #__VA_ARGS__            \
                 << " need to be accessible on the GPU."; \
    return;                                               \
  }

}  // namespace pynvblox

#include "nvblox_torch/impl/check_utils_impl.h"
