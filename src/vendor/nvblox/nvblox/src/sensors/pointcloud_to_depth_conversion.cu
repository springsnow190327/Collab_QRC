/*
Copyright 2025 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <limits>

#include "nvblox/core/internal/cuda/atomic_float.cuh"
#include "nvblox/core/internal/error_check.h"
#include "nvblox/sensors/pointcloud.h"
#include "nvblox/utils/cuda_kernel_utils.h"
#include "nvblox/utils/timing.h"

namespace nvblox {

// Helper kernels for pointcloud to depth image conversion.

__global__ void initDepthImageKernel(const int rows, const int cols,
                                     const float init_value,
                                     float* depth_image) {
  const int col = blockIdx.x * blockDim.x + threadIdx.x;
  const int row = blockIdx.y * blockDim.y + threadIdx.y;

  if (col >= cols || row >= rows) {
    return;
  }

  image::access(row, col, cols, depth_image) = init_value;
}

__global__ void setSentinelDepthToZeroKernel(const int rows, const int cols,
                                             const float sentinel_value,
                                             float* depth_image) {
  const int col = blockIdx.x * blockDim.x + threadIdx.x;
  const int row = blockIdx.y * blockDim.y + threadIdx.y;

  if (col >= cols || row >= rows) {
    return;
  }

  float& depth = image::access(row, col, cols, depth_image);
  if (fabsf(depth - sentinel_value) < 1e-6) {
    depth = 0.0f;
  }
}

}  // namespace nvblox
