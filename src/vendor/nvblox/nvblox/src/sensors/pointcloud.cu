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

Pointcloud::Pointcloud(int size, MemoryType memory_type, bool init_timestamps)
    : points_(unified_vector<Vector3f>(size, memory_type)),
      timestamps_ms_(init_timestamps ? std::make_optional<unified_vector<Time>>(
                                           size, memory_type)
                                     : std::nullopt) {}

Pointcloud::Pointcloud(MemoryType memory_type, bool init_timestamps)
    // zero-sized allocation in order to store memory_type in the underlying
    // vectors
    : Pointcloud(0, memory_type, init_timestamps) {}

void Pointcloud::copyFromAsync(const Pointcloud& other,
                               const CudaStream& cuda_stream) {
  points_.copyFromAsync(other.points(), cuda_stream);
  // Copy timestamps if they exist in the source pointcloud
  if (other.timestamps_ms().has_value()) {
    if (!timestamps_ms_.has_value()) {
      timestamps_ms_ = unified_vector<Time>(points_.memory_type());
    }
    timestamps_ms_->copyFromAsync(*other.timestamps_ms(), cuda_stream);
  }
}

void Pointcloud::copyPointsFromAsync(const std::vector<Vector3f>& points,
                                     const CudaStream& cuda_stream) {
  points_.copyFromAsync(points, cuda_stream);
}

void Pointcloud::copyPointsFromAsync(const unified_vector<Vector3f>& points,
                                     const CudaStream& cuda_stream) {
  points_.copyFromAsync(points, cuda_stream);
}

void Pointcloud::copyTimestampsFromAsync(
    const unified_vector<Time>& timestamps_ms, const CudaStream& cuda_stream) {
  if (!timestamps_ms_.has_value()) {
    // Initialize the timestamps vector if it doesn't exist
    timestamps_ms_ = unified_vector<Time>(timestamps_ms.memory_type());
  }
  timestamps_ms_->copyFromAsync(timestamps_ms, cuda_stream);
}

void Pointcloud::copyTimestampsFromAsync(const std::vector<Time>& timestamps_ms,
                                         const CudaStream& cuda_stream) {
  if (!timestamps_ms_.has_value()) {
    // Initialize the timestamps vector if it doesn't exist
    timestamps_ms_ = unified_vector<Time>(points_.memory_type());
  }
  timestamps_ms_->copyFromAsync(timestamps_ms, cuda_stream);
}

void Pointcloud::resizeAsync(int size, const CudaStream& cuda_stream) {
  points_.resizeAsync(size, cuda_stream);
  // Keep timestamps synchronized if they exist
  if (timestamps_ms_.has_value()) {
    timestamps_ms_->resizeAsync(size, cuda_stream);
  }
}

void Pointcloud::resize(int size) { resizeAsync(size, CudaStreamOwning()); }

void Pointcloud::reserveAsync(int size, const CudaStream& cuda_stream) {
  points_.reserveAsync(size, cuda_stream);
  // Keep timestamps synchronized if they exist
  if (timestamps_ms_.has_value()) {
    timestamps_ms_->reserveAsync(size, cuda_stream);
  }
}

void Pointcloud::reserve(int size) { reserveAsync(size, CudaStreamOwning()); }

const std::optional<unified_vector<Time>>& Pointcloud::timestamps_ms() const {
  checkTimestampsConsistency();
  return timestamps_ms_;
}

std::optional<unified_vector<Time>>& Pointcloud::timestamps_ms() {
  checkTimestampsConsistency();
  return timestamps_ms_;
}

Time* Pointcloud::timestampsPtr() {
  checkTimestampsConsistency();
  return timestamps_ms_.has_value() ? timestamps_ms_->data() : nullptr;
}

const Time* Pointcloud::timestampsConstPtr() const {
  checkTimestampsConsistency();
  return timestamps_ms_.has_value() ? timestamps_ms_->data() : nullptr;
}

void Pointcloud::checkTimestampsConsistency() const {
  if (timestamps_ms_.has_value()) {
    CHECK_EQ(timestamps_ms_->size(), points_.size())
        << "Timestamp count must match point count";
  }
}

// Pointcloud operations

__global__ void transformPointcloudKernel(const Transform T_out_in,
                                          int pointcloud_size,
                                          const Vector3f* pointcloud_in,
                                          Vector3f* pointcloud_out) {
  const int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index >= pointcloud_size) {
    return;
  }

  pointcloud_out[index] = T_out_in * pointcloud_in[index];
}

void transformPointcloudOnGPU(const Transform& T_out_in,
                              const Pointcloud& pointcloud_in,
                              Pointcloud* pointcloud_out_ptr) {
  // Calls the streamed version after creating a stream
  CudaStreamOwning cuda_stream;
  transformPointcloudOnGPU(T_out_in, pointcloud_in, pointcloud_out_ptr,
                           &cuda_stream);
}

void transformPointcloudOnGPU(const Transform& T_out_in,
                              const Pointcloud& pointcloud_in,
                              Pointcloud* pointcloud_out_ptr,
                              CudaStream* cuda_stream_ptr) {
  CHECK_NOTNULL(pointcloud_out_ptr);
  CHECK_NOTNULL(cuda_stream_ptr);
  CHECK(pointcloud_out_ptr->memory_type() == MemoryType::kDevice ||
        pointcloud_out_ptr->memory_type() == MemoryType::kUnified);

  if (pointcloud_in.empty()) {
    return;
  }
  pointcloud_out_ptr->resizeAsync(pointcloud_in.size(), *cuda_stream_ptr);

  constexpr int kThreadsPerThreadBlock = 512;
  const int num_blocks(
      divideRoundUp(pointcloud_in.size(), kThreadsPerThreadBlock));
  transformPointcloudKernel<<<num_blocks, kThreadsPerThreadBlock, 0,
                              *cuda_stream_ptr>>>(
      T_out_in, pointcloud_in.size(), pointcloud_in.pointsConstPtr(),
      pointcloud_out_ptr->pointsPtr());
  cuda_stream_ptr->synchronize();
  checkCudaErrors(cudaPeekAtLastError());
}

}  // namespace nvblox
