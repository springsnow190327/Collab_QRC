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
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <chrono>
#include <numeric>
#include <thread>

#include "nvblox/core/array.h"
#include "nvblox/core/unified_ptr.h"

constexpr size_t kCapacity = 7;
using namespace nvblox;

// Kernel used for testing
template <typename FunctorType>
__global__ void testKernel() {
  FunctorType()();
}

// Run functor on GPU
template <typename FunctorType>
void runOnGpu() {
  testKernel<FunctorType><<<1, 1, 0, CudaStreamOwning()>>>();
}

// Run functor on CPU
template <typename FunctorType>
void runOnCpu() {
  FunctorType()();
}

// Test default constructor
struct TestDefaultConstruct {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array;
    for (size_t i = 0; i < kCapacity; ++i) {
      // Note that we can't use EXPECT_*() since it only works on CPU.
      NVBLOX_CHECK(array[i] == 0, "Array not zero-initialized");
    }
  }
};
TEST(Array, TestDefaultConstructCpu) { runOnCpu<TestDefaultConstruct>(); }
TEST(Array, TestDefaultConstructGpu) { runOnGpu<TestDefaultConstruct>(); }

// Test data getter
struct TestData {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array;
    NVBLOX_CHECK(array.data() != nullptr, "Data is null");
  }
};
TEST(Array, TestDataCpu) { runOnCpu<TestData>(); }
TEST(Array, TestDatasGpu) { runOnGpu<TestData>(); }

// Test size getter
struct TestSize {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array;
    NVBLOX_CHECK(array.size() == kCapacity, "Incorrect size");
  }
};
TEST(Array, TestSizeCpu) { runOnCpu<TestSize>(); }
TEST(Array, TestSizesGpu) { runOnGpu<TestSize>(); }

// Test accessor
struct TestAccess {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array;
    constexpr size_t kIndex = kCapacity / 2;
    NVBLOX_CHECK(array[kIndex] == 0, "Expected zero");
    array[kIndex] = 3;
    NVBLOX_CHECK(array[kIndex] == 3, "Expected nonzero");
  }
};
TEST(Array, TestAccessCpu) { runOnCpu<TestAccess>(); }
TEST(Array, TestAccesssGpu) { runOnGpu<TestAccess>(); }

// Test iterators
struct TestIterators {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array;
    for (size_t i = 0; i < array.size(); ++i) {
      array[i] = i;
    }
    int expected = 0;
    for (auto itr = array.begin(); itr != array.end(); ++itr) {
      NVBLOX_CHECK(*itr == expected++, "Unexpected value");
    }
  }
};
TEST(Array, TestIteratorsCpu) { runOnCpu<TestIterators>(); }
TEST(Array, TestIteratorsGpu) { runOnGpu<TestIterators>(); }

// Test const iterators
struct TestConstIterators {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array;
    for (size_t i = 0; i < array.size(); ++i) {
      array[i] = i;
    }
    int expected = 0;
    for (auto itr = array.cbegin(); itr != array.cend(); ++itr) {
      NVBLOX_CHECK(*itr == expected++, "Unexpected value");
    }
  }
};
TEST(Array, TestConstIteratorsCpu) { runOnCpu<TestConstIterators>(); }
TEST(Array, TestConstIteratorsGpu) { runOnGpu<TestConstIterators>(); }

// Test range based loop
struct TestRangeBasedLoop {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array({0, 1, 2, 3, 4, 5, 6});
    int expected = 0;
    for (auto item : array) {
      NVBLOX_CHECK(item == expected++, "Unexpected value");
    }
  }
};
TEST(Array, TestRangeBasedLoopCpu) { runOnCpu<TestRangeBasedLoop>(); }
TEST(Array, TestRangeBasedLoopGpu) { runOnGpu<TestRangeBasedLoop>(); }

// Test range based const loop
struct TestRangeBasedConstLoop {
  __host__ __device__ void operator()() {
    const Array<int, kCapacity> array({0, 1, 2, 3, 4, 5, 6});
    int expected = 0;
    for (const auto& item : array) {
      NVBLOX_CHECK(item == expected++, "Unexpected value");
    }
  }
};
TEST(Array, TestRangeBasedConstLoopCpu) { runOnCpu<TestRangeBasedConstLoop>(); }
TEST(Array, TestRangeBasedConstLoopGpu) { runOnGpu<TestRangeBasedConstLoop>(); }
// Test assign construct
struct TestAssignConstruct {
  __host__ __device__ void operator()() {
    Array<int, kCapacity> array({0, 1, 2, 3, 4, 5, 6});
    auto array_assigned = array;
    int expected = 0;
    NVBLOX_CHECK(array_assigned.size() == array.size(), "Size mismatch");
    for (size_t i = 0; i < array_assigned.size(); ++i) {
      NVBLOX_CHECK(array_assigned[i] == expected++, "Unexpected value");
    }
  }
};
TEST(Array, TestAssignConstructCpu) { runOnCpu<TestAssignConstruct>(); }
TEST(Array, TestAssignConstructGpu) { runOnGpu<TestAssignConstruct>(); }

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
