// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// RAII wrappers for CUDA device + pinned-host memory and streams. The goal
// is total lifecycle hygiene: every cudaMalloc paired with cudaFree, every
// cudaStreamCreate paired with cudaStreamDestroy, every cudaHostAlloc
// paired with cudaFreeHost — automatically, even on exception during
// construction of a containing object (ctor-half-failure case).
//
// Inspired by std::unique_ptr but with cuda-specific deleters. Move-only.

#ifndef NAV_ALGO_MPPI_CUDA__DEVICE_MEMORY_HPP_
#define NAV_ALGO_MPPI_CUDA__DEVICE_MEMORY_HPP_

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include <cuda_runtime.h>

namespace nav_algo_mppi_cuda
{

inline void throwIfCudaError(cudaError_t e, const char * msg)
{
  if (e != cudaSuccess) {
    throw std::runtime_error(
      std::string("CUDA error ") + msg + ": " + cudaGetErrorString(e));
  }
}

// Reset CUDA's sticky error state. Call after kernel launches to prevent a
// failed launch from poisoning every subsequent CUDA API call in the process.
inline void clearStickyCudaError() noexcept
{
  (void)cudaGetLastError();
}

// Owns a cudaMalloc'd device buffer of `count` elements of type T. Free
// happens in the dtor; never leaks even on partial-construction failure of
// the containing object (a thrown ctor unwinds member dtors).
template <typename T>
class DevicePtr
{
public:
  DevicePtr() = default;

  explicit DevicePtr(size_t count)
  : count_(count)
  {
    if (count == 0) return;
    throwIfCudaError(
      cudaMalloc(reinterpret_cast<void **>(&ptr_), count * sizeof(T)),
      "cudaMalloc");
  }

  ~DevicePtr() { reset(); }

  DevicePtr(const DevicePtr &) = delete;
  DevicePtr & operator=(const DevicePtr &) = delete;

  DevicePtr(DevicePtr && o) noexcept : ptr_(o.ptr_), count_(o.count_)
  {
    o.ptr_ = nullptr;
    o.count_ = 0;
  }
  DevicePtr & operator=(DevicePtr && o) noexcept
  {
    if (this != &o) {
      reset();
      ptr_ = o.ptr_;
      count_ = o.count_;
      o.ptr_ = nullptr;
      o.count_ = 0;
    }
    return *this;
  }

  void reset() noexcept
  {
    if (ptr_) {
      // Suppress error: we're in a dtor path; reporting would either
      // re-throw (forbidden) or get lost. CUDA leaks here would be a
      // shutdown-only condition anyway.
      cudaFree(ptr_);
      ptr_ = nullptr;
    }
    count_ = 0;
  }

  T * get() noexcept { return ptr_; }
  const T * get() const noexcept { return ptr_; }
  size_t count() const noexcept { return count_; }
  size_t bytes() const noexcept { return count_ * sizeof(T); }

  // Implicit decay for ergonomic kernel-launch site usage.
  operator T *() noexcept { return ptr_; }
  operator const T *() const noexcept { return ptr_; }

private:
  T * ptr_{nullptr};
  size_t count_{0};
};

// Owns a cudaHostAlloc'd page-locked host buffer. Required for true async
// DMA: pageable memory forces cudaMemcpyAsync to fall back to a sync copy.
template <typename T>
class HostPinnedPtr
{
public:
  HostPinnedPtr() = default;

  explicit HostPinnedPtr(size_t count, unsigned int flags = cudaHostAllocDefault)
  : count_(count)
  {
    if (count == 0) return;
    throwIfCudaError(
      cudaHostAlloc(reinterpret_cast<void **>(&ptr_), count * sizeof(T), flags),
      "cudaHostAlloc");
  }

  ~HostPinnedPtr() { reset(); }

  HostPinnedPtr(const HostPinnedPtr &) = delete;
  HostPinnedPtr & operator=(const HostPinnedPtr &) = delete;

  HostPinnedPtr(HostPinnedPtr && o) noexcept : ptr_(o.ptr_), count_(o.count_)
  {
    o.ptr_ = nullptr;
    o.count_ = 0;
  }
  HostPinnedPtr & operator=(HostPinnedPtr && o) noexcept
  {
    if (this != &o) {
      reset();
      ptr_ = o.ptr_;
      count_ = o.count_;
      o.ptr_ = nullptr;
      o.count_ = 0;
    }
    return *this;
  }

  void reset() noexcept
  {
    if (ptr_) {
      cudaFreeHost(ptr_);
      ptr_ = nullptr;
    }
    count_ = 0;
  }

  T * get() noexcept { return ptr_; }
  const T * get() const noexcept { return ptr_; }
  size_t count() const noexcept { return count_; }
  size_t bytes() const noexcept { return count_ * sizeof(T); }

private:
  T * ptr_{nullptr};
  size_t count_{0};
};

// Owns a cudaStreamCreate'd stream. Per-CudaBackend isolation: multiple
// concurrent MPPI instances (e.g. dual-robot) won't serialize through the
// default stream.
class Stream
{
public:
  Stream() { throwIfCudaError(cudaStreamCreate(&s_), "cudaStreamCreate"); }
  ~Stream()
  {
    if (s_) {
      // Ensure no in-flight work before destroying — guards against the
      // pathological case where a kernel was launched but not synced.
      (void)cudaStreamSynchronize(s_);
      cudaStreamDestroy(s_);
    }
  }
  Stream(const Stream &) = delete;
  Stream & operator=(const Stream &) = delete;
  Stream(Stream && o) noexcept : s_(o.s_) { o.s_ = nullptr; }
  Stream & operator=(Stream && o) noexcept
  {
    if (this != &o) {
      if (s_) {
        (void)cudaStreamSynchronize(s_);
        cudaStreamDestroy(s_);
      }
      s_ = o.s_;
      o.s_ = nullptr;
    }
    return *this;
  }
  cudaStream_t get() const noexcept { return s_; }
  operator cudaStream_t() const noexcept { return s_; }

private:
  cudaStream_t s_{nullptr};
};

}  // namespace nav_algo_mppi_cuda

#endif  // NAV_ALGO_MPPI_CUDA__DEVICE_MEMORY_HPP_
