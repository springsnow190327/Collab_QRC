/*
Copyright 2024 NVIDIA CORPORATION

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
#pragma once

#include <cuda_runtime.h>
#include <cstdint>

#include "nvblox/core/internal/error_check.h"

namespace nvblox {

/// Iterator class for Array
template <typename T, size_t Capacity, bool IsConst>
class ArrayIterator {
 public:
  /// Some traits needed for this to become an iterator
  using iterator_category = std::forward_iterator_tag;
  using value_type = T;
  using difference_type = std::ptrdiff_t;

  /// Pointer and reference types depends on if this is a const iterator or not
  using pointer = typename std::conditional<IsConst, const T*, T*>::type;
  using reference = typename std::conditional<IsConst, const T&, T&>::type;

  /// Construct from a pointer
  __host__ __device__ ArrayIterator(pointer ptr) : ptr_(ptr) {}
  __host__ __device__ ~ArrayIterator() = default;

  /// Accessors
  __host__ __device__ reference operator*() { return *ptr_; }
  __host__ __device__ reference operator*() const { return *ptr_; }
  __host__ __device__ pointer operator->() const { return ptr_; }

  /// Prefix increment
  __host__ __device__ ArrayIterator operator++() {
    ++ptr_;
    return *this;
  }

  /// Postfix increment
  __host__ __device__ ArrayIterator operator++(int) {
    ArrayIterator tmp = *this;
    ++(*this);
    return tmp;
  }

  /// Comparison operators
  __host__ __device__ friend bool operator==(const ArrayIterator& a,
                                             const ArrayIterator& b) {
    return a.ptr_ == b.ptr_;
  };
  __host__ __device__ friend bool operator!=(const ArrayIterator& a,
                                             const ArrayIterator& b) {
    return a.ptr_ != b.ptr_;
  };

 private:
  pointer ptr_;
};

/// Class that mimics std::array but supports device code. Data is stored in a
/// POD (c-style) array.
template <typename T, size_t Capacity>
class Array {
  static_assert(Capacity > 0, "Zero capacity array is not allowed");

 public:
  using value_type = T;
  /// Default constructor. Initializes the array to zero
  /// Note that we need to use 0.F here since cuda 11.8 doesn't support implicit
  /// conversion between int and __half.
  __host__ __device__ Array() : data_{static_cast<T>(0.F)} {}

  /// Array initialization
  __host__ __device__ Array(const T (&init_array)[Capacity]) {
    for (size_t i = 0; i < Capacity; ++i) {
      data_[i] = init_array[i];
    }
  }

  /// Return a pointer to the beginning of the array.
  __host__ __device__ T* data() { return data_; }
  __host__ __device__ const T* data() const { return data_; }

  /// Return the size (=capacity) of the array.
  __host__ __device__ static constexpr size_t size() { return Capacity; }

  /// Element access
  __host__ __device__ T operator[](size_t index) const {
    NVBLOX_DCHECK(index < size(), "Index out of bounds");
    return data_[index];
  }
  __host__ __device__ T& operator[](size_t index) {
    NVBLOX_DCHECK(index < size(), "Index out of bounds");
    return data_[index];
  }

  /// Iterator types
  using iterator = ArrayIterator<T, Capacity, false>;
  using const_iterator = ArrayIterator<T, Capacity, true>;

  /// Get an iterator to the first voxel
  __host__ __device__ iterator begin() { return iterator(data_); }
  __host__ __device__ const_iterator begin() const { return cbegin(); }
  __host__ __device__ const_iterator cbegin() const {
    return const_iterator(data_);
  }

  /// Get an iterator to the past-the-end voxel
  __host__ __device__ iterator end() { return iterator(data_ + Capacity); }
  __host__ __device__ const_iterator end() const { return cend(); }
  __host__ __device__ const_iterator cend() const {
    return const_iterator(data_ + Capacity);
  }

 protected:
  T data_[Capacity];
};

}  // namespace nvblox
