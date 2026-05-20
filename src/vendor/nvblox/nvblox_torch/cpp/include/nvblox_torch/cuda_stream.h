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

#include <c10/cuda/CUDAStream.h>

#include "nvblox/core/cuda_stream.h"

namespace pynvblox {

/// This class wraps a CUDA stream in an nvblox compatible interface.
/// This is necessary because at::cuda::getCurrentCUDAStream() returns a
/// stream by value, which we cant wrap with our Cuda stream classes in
/// nvblox core.
class NvbloxTorchCudaStream : public nvblox::CudaStream {
 public:
  NvbloxTorchCudaStream(cudaStream_t raw_stream);
  virtual ~NvbloxTorchCudaStream() = default;

  /// Returns the underlying CUDA stream
  /// @return The raw CUDA stream
  cudaStream_t& get();
  const cudaStream_t& get() const;

  operator cudaStream_t();
  operator cudaStream_t() const;

  /// Synchronize the stream
  void synchronize() const;

 protected:
  cudaStream_t raw_stream_;
};

NvbloxTorchCudaStream getCurrentStream();

}  // namespace pynvblox
