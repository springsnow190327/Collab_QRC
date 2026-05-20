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

#include <nvblox/core/internal/error_check.h>

#include "nvblox_torch/cuda_stream.h"

namespace pynvblox {

NvbloxTorchCudaStream::NvbloxTorchCudaStream(cudaStream_t raw_stream)
    : raw_stream_(raw_stream) {}

cudaStream_t& NvbloxTorchCudaStream::get() { return raw_stream_; }

const cudaStream_t& NvbloxTorchCudaStream::get() const { return raw_stream_; }

NvbloxTorchCudaStream::operator cudaStream_t() { return raw_stream_; }

NvbloxTorchCudaStream::operator cudaStream_t() const { return raw_stream_; }

void NvbloxTorchCudaStream::synchronize() const {
  checkCudaErrors(cudaStreamSynchronize(raw_stream_));
}

NvbloxTorchCudaStream getCurrentStream() {
  return NvbloxTorchCudaStream(at::cuda::getCurrentCUDAStream());
}

}  // namespace pynvblox
