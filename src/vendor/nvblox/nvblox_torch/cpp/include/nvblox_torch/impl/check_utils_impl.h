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

#include <optional>

#include <torch/script.h>

namespace pynvblox {

template <typename T>
inline bool checkOnGPU(const T& tensor) {
  return tensor.device().is_cuda();
}

// Specialization for std::optional.
template <typename T>
inline bool checkOnGPU(const std::optional<T>& maybe_tensor) {
  // If the optional is not set, we return true as the tensor isn't used
  // and therefore doesn't matter if it's on the GPU or not.
  // If it is set, we check if it's on the GPU.
  return !maybe_tensor.has_value() || checkOnGPU(maybe_tensor.value());
}

template <typename... Args>
inline bool checkAllOnGPU(const Args&... args) {
  return (checkOnGPU(args) && ...);
}

template <size_t N>
inline bool checkSizes(const torch::Tensor& tensor, const int (&sizes)[N]) {
  if (tensor.dim() != N) {
    return false;
  }
  for (size_t i = 0; i < N; i++) {
    if (sizes[i] >= 0) {
      if (tensor.sizes()[i] != sizes[i]) {
        return false;
      }
    }
  }
  return true;
}

template <typename T>
inline bool checkElementSize(const torch::Tensor& tensor) {
  return tensor.element_size() == sizeof(T);
}

inline bool checkSizesEqual(const torch::Tensor& tensor_1,
                            const torch::Tensor& tensor_2) {
  if (tensor_1.dim() != tensor_2.dim()) {
    return false;
  }
  for (int i = 0; i < tensor_1.dim(); i++) {
    if (tensor_1.sizes()[i] != tensor_2.sizes()[i]) {
      return false;
    }
  }
  return true;
}

inline bool checkImageDimensionsEqual(const torch::Tensor& tensor_1,
                                      const torch::Tensor& tensor_2) {
  return tensor_1.sizes()[0] == tensor_2.sizes()[0] &&
         tensor_1.sizes()[1] == tensor_2.sizes()[1];
}

}  // namespace pynvblox
