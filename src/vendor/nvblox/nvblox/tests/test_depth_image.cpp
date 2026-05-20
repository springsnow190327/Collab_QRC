/*
Copyright 2022 NVIDIA CORPORATION

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
#include <gtest/gtest.h>

#include "nvblox/core/types.h"
#include "nvblox/sensors/image.h"
#include "nvblox/tests/gpu_image_routines.h"
#include "nvblox/tests/utils.h"

using namespace nvblox;

constexpr float kFloatEpsilon = 1e-4;

class DepthImageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Uninitialized depth frame
    depth_frame_ = DepthImage(rows_, cols_, MemoryType::kUnified);
  }

  Index2D getRandomPixel() {
    return Index2D(test_utils::randomIntInRange(0., cols_ - 1),
                   test_utils::randomIntInRange(0, rows_ - 1));
  }

  int rows_ = 480;
  int cols_ = 640;
  DepthImage depth_frame_{MemoryType::kDevice};
};

void setImageConstantOnCpu(const float value, DepthImage* depth_frame_ptr) {
  // Set everything to 1.0 through one access method and check through the other
  for (int row_idx = 0; row_idx < depth_frame_ptr->rows(); row_idx++) {
    for (int col_idx = 0; col_idx < depth_frame_ptr->cols(); col_idx++) {
      for (int element_idx = 0;
           element_idx < depth_frame_ptr->num_elements_per_pixel();
           element_idx++) {
        (*depth_frame_ptr)(row_idx, col_idx) = value;
      }
    }
  }
}

TEST_F(DepthImageTest, Host) {
  // Set constant on CPU
  setImageConstantOnCpu(1.0, &depth_frame_);

  // Check on the CPU
  for (int lin_idx = 0; lin_idx < depth_frame_.numel(); lin_idx++) {
    EXPECT_EQ(depth_frame_(lin_idx), 1.0f);
  }
}

TEST_F(DepthImageTest, Device) {
  // Set constant on GPU
  constexpr float kPixelValue = 1.0f;
  test_utils::setImageConstantOnGpu(kPixelValue, &depth_frame_);

  // Check on the CPU
  for (int lin_idx = 0; lin_idx < depth_frame_.numel(); lin_idx++) {
    EXPECT_EQ(depth_frame_(lin_idx), 1.0f);
  }
}

TEST_F(DepthImageTest, DeviceReduction) {
  // Make sure this is deterministic.
  std::srand(0);

  // Set constant on CPU
  constexpr float kPixelValue = 1.0f;
  setImageConstantOnCpu(kPixelValue, &depth_frame_);

  // Change a single value on the CPU
  constexpr float kMaxValue = 100.0f;
  constexpr float kMinValue = -100.0f;
  const Index2D u_max = getRandomPixel();
  Index2D u_min = u_max;
  while ((u_min.array() == u_max.array()).all()) {
    u_min = getRandomPixel();
  }
  depth_frame_(u_max.y(), u_max.x()) = kMaxValue;
  depth_frame_(u_min.y(), u_min.x()) = kMinValue;

  // Reduction on the GPU
  const float max = image::maxGPU(depth_frame_, CudaStreamOwning());
  const float min = image::minGPU(depth_frame_, CudaStreamOwning());
  float minmax_min;
  float minmax_max;
  image::minmaxGPU(depth_frame_, &minmax_min, &minmax_max, CudaStreamOwning());

  // Check on the CPU
  EXPECT_EQ(max, kMaxValue);
  EXPECT_EQ(min, kMinValue);
  EXPECT_EQ(minmax_min, kMinValue);
  EXPECT_EQ(minmax_max, kMaxValue);
}

TEST_F(DepthImageTest, GpuOperation) {
  // Set constant on CPU
  constexpr float kPixelValue = 1.0f;
  setImageConstantOnCpu(kPixelValue, &depth_frame_);

  // Element wise min
  image::elementWiseMinInPlaceGPUAsync(0.5f, &depth_frame_, CudaStreamOwning());

  // Reduction on the GPU
  const float max = image::maxGPU(depth_frame_, CudaStreamOwning());
  EXPECT_EQ(max, 0.5f);

  // Element wise max
  image::elementWiseMaxInPlaceGPUAsync(1.5f, &depth_frame_, CudaStreamOwning());

  const float min = image::minGPU(depth_frame_, CudaStreamOwning());
  EXPECT_EQ(min, 1.5f);
}

TEST_F(DepthImageTest, CopyFrom) {
  constexpr float kConstant{3};
  std::vector<float> buffer(rows_ * cols_, kConstant);

  DepthImage image(MemoryType::kHost);
  image.copyFrom(rows_, cols_, buffer.data());
  EXPECT_EQ(image.rows(), rows_);
  EXPECT_EQ(image.cols(), cols_);

  for (int i = 0; i < image.rows() * image.cols(); ++i) {
    EXPECT_EQ(image(i), kConstant);
  }
}

TEST_F(DepthImageTest, DeepCopy) {
  // Set constant on CPU
  constexpr float kPixelValue = 1.0f;
  setImageConstantOnCpu(kPixelValue, &depth_frame_);

  // Copy
  DepthImage copy(MemoryType::kHost);
  copy.copyFrom(depth_frame_);

  // Check the copy is actually a copy
  for (int lin_idx = 0; lin_idx < copy.numel(); lin_idx++) {
    EXPECT_EQ(copy(lin_idx), kPixelValue);
  }
}

TEST_F(DepthImageTest, DifferenceImage) {
  DepthImage image_1(2, 2, MemoryType::kUnified);
  image_1(0, 0) = 3.0f;
  image_1(0, 1) = 3.0f;
  image_1(1, 0) = 3.0f;
  image_1(1, 1) = 3.0f;

  DepthImage image_2(2, 2, MemoryType::kUnified);
  image_2(0, 0) = 2.0f;
  image_2(0, 1) = 2.0f;
  image_2(1, 0) = 2.0f;
  image_2(1, 1) = 2.0f;

  DepthImage diff_image{MemoryType::kDevice};

  image::getDifferenceImageGPUAsync(image_1, image_2, &diff_image,
                                    CudaStreamOwning());

  // Check the function allocated the output
  EXPECT_EQ(diff_image.rows(), 2);
  EXPECT_EQ(diff_image.cols(), 2);

  // Check the multiplication actually worked.
  EXPECT_NEAR(diff_image(0, 0), 1.0f, kFloatEpsilon);
  EXPECT_NEAR(diff_image(0, 1), 1.0f, kFloatEpsilon);
  EXPECT_NEAR(diff_image(1, 0), 1.0f, kFloatEpsilon);
  EXPECT_NEAR(diff_image(1, 1), 1.0f, kFloatEpsilon);
}

TEST_F(DepthImageTest, ImageMultiplication) {
  DepthImage image(2, 2, MemoryType::kUnified);
  image(0, 0) = 1.0f;
  image(0, 1) = 2.0f;
  image(1, 0) = 3.0f;
  image(1, 1) = 4.0f;

  image::elementWiseMultiplicationInPlaceGPUAsync(2.0f, &image,
                                                  CudaStreamOwning());

  // Check the difference actually worked.
  EXPECT_NEAR(image(0, 0), 2.0f, kFloatEpsilon);
  EXPECT_NEAR(image(0, 1), 4.0f, kFloatEpsilon);
  EXPECT_NEAR(image(1, 0), 6.0f, kFloatEpsilon);
  EXPECT_NEAR(image(1, 1), 8.0f, kFloatEpsilon);
}

TEST_F(DepthImageTest, ImageCast) {
  DepthImage image(2, 2, MemoryType::kUnified);
  image(0, 0) = 1.1f;
  image(0, 1) = 2.1f;
  image(1, 0) = 3.1f;
  image(1, 1) = 4.1f;

  MonoImage image_out(MemoryType::kDevice);
  image::castGPUAsync(image, &image_out, CudaStreamOwning());

  EXPECT_EQ(image_out(0, 0), 1);
  EXPECT_EQ(image_out(0, 1), 2);
  EXPECT_EQ(image_out(1, 0), 3);
  EXPECT_EQ(image_out(1, 1), 4);
}

TEST_F(DepthImageTest, ImageView) {
  // Mock a external image buffer
  const int image_buffer[] = {1, 2, 3, 4};

  const int kRows = 2;
  const int kCols = 2;
  ImageView<const int> image_view(kRows, kCols, image_buffer);

  EXPECT_EQ(image_view(0, 0), 1);
  EXPECT_EQ(image_view(0, 1), 2);
  EXPECT_EQ(image_view(1, 0), 3);
  EXPECT_EQ(image_view(1, 1), 4);

  // Comment check (doesn't compile)
  // Can't assign onto const buffer
  // image_view(0,0) = 5;

  int mutable_image_buffer[] = {1, 2, 3, 4};

  ImageView<int> mutable_image_view(kRows, kCols, mutable_image_buffer);

  EXPECT_EQ(mutable_image_view(0, 0), 1);
  EXPECT_EQ(mutable_image_view(0, 1), 2);
  EXPECT_EQ(mutable_image_view(1, 0), 3);
  EXPECT_EQ(mutable_image_view(1, 1), 4);

  // Change the buffer through the image view
  mutable_image_view(0, 0) = 5;
  mutable_image_view(0, 1) = 6;
  mutable_image_view(1, 0) = 7;
  mutable_image_view(1, 1) = 8;

  EXPECT_EQ(mutable_image_buffer[0], 5);
  EXPECT_EQ(mutable_image_buffer[1], 6);
  EXPECT_EQ(mutable_image_buffer[2], 7);
  EXPECT_EQ(mutable_image_buffer[3], 8);

  // Copy the image view
  ImageView<int> mutable_image_view_copy(mutable_image_view);

  EXPECT_EQ(mutable_image_view(0, 0), 5);
  EXPECT_EQ(mutable_image_view(0, 1), 6);
  EXPECT_EQ(mutable_image_view(1, 0), 7);
  EXPECT_EQ(mutable_image_view(1, 1), 8);

  mutable_image_view_copy(0, 0) = 9;
  mutable_image_view_copy(0, 1) = 10;
  mutable_image_view_copy(1, 0) = 11;
  mutable_image_view_copy(1, 1) = 12;

  EXPECT_EQ(mutable_image_buffer[0], 9);
  EXPECT_EQ(mutable_image_buffer[1], 10);
  EXPECT_EQ(mutable_image_buffer[2], 11);
  EXPECT_EQ(mutable_image_buffer[3], 12);

  // Copy the ImageView using the copy assignment operator
  ImageView<int> mutable_image_view_copy_2(mutable_image_view);

  mutable_image_view_copy_2(0, 0) = 13;
  mutable_image_view_copy_2(0, 1) = 14;
  mutable_image_view_copy_2(1, 0) = 15;
  mutable_image_view_copy_2(1, 1) = 16;

  EXPECT_EQ(mutable_image_buffer[0], 13);
  EXPECT_EQ(mutable_image_buffer[1], 14);
  EXPECT_EQ(mutable_image_buffer[2], 15);
  EXPECT_EQ(mutable_image_buffer[3], 16);
}

TEST_F(DepthImageTest, SetZero) {
  const uint8_t image_buffer[] = {0, 1, 2, 3, 4, 5};
  MonoImage image(MemoryType::kUnified);
  image.copyFrom(2, 3, image_buffer);

  for (int i = 0; i < 6; i++) {
    EXPECT_EQ(image(i), i);
  }

  image.setZeroAsync(CudaStreamOwning());

  for (int i = 0; i < 6; i++) {
    EXPECT_EQ(image(i), 0);
  }
}

TEST_F(DepthImageTest, ImageViewFromImage) {
  //
  MonoImage image(2, 2, MemoryType::kUnified);
  image(0, 0) = 0;
  image(0, 1) = 1;
  image(1, 0) = 2;
  image(1, 1) = 3;

  MonoImageView image_view(image);

  image_view(0, 0) = 4;
  image_view(0, 1) = 5;
  image_view(1, 0) = 6;
  image_view(1, 1) = 7;

  EXPECT_EQ(image(0, 0), 4);
  EXPECT_EQ(image(0, 1), 5);
  EXPECT_EQ(image(1, 0), 6);
  EXPECT_EQ(image(1, 1), 7);

  const MonoImage& const_image = image;

  ImageView<const uint8_t> const_image_view(const_image);

  EXPECT_EQ(image(0, 0), 4);
  EXPECT_EQ(image(0, 1), 5);
  EXPECT_EQ(image(1, 0), 6);
  EXPECT_EQ(image(1, 1), 7);

  // NOTE(alex.millane): The following should not compile because you can't
  // write to a const view.
  // const_image_view(0, 0) = 8;
  // const_image_view(0, 1) = 9;
  // const_image_view(1, 0) = 10;
  // const_image_view(1, 1) = 11;
}

TEST_F(DepthImageTest, ElementWiseBinaryOps) {
  MonoImage image_1(2, 2, MemoryType::kUnified);
  image_1(0, 0) = 0;
  image_1(0, 1) = 1;
  image_1(1, 0) = 0;
  image_1(1, 1) = 1;

  MonoImage image_2(2, 2, MemoryType::kUnified);
  image_2(0, 0) = 0;
  image_2(0, 1) = 0;
  image_2(1, 0) = 1;
  image_2(1, 1) = 1;

  image::elementWiseMaxInPlaceGPUAsync(image_1, &image_2, CudaStreamOwning());

  EXPECT_EQ(image_2(0, 0), 0);
  EXPECT_EQ(image_2(0, 1), 1);
  EXPECT_EQ(image_2(1, 0), 1);
  EXPECT_EQ(image_2(1, 1), 1);

  MonoImage image_3(2, 2, MemoryType::kUnified);
  image_3(0, 0) = 0;
  image_3(0, 1) = 0;
  image_3(1, 0) = 1;
  image_3(1, 1) = 1;

  image::elementWiseMinInPlaceGPUAsync(image_1, &image_3, CudaStreamOwning());

  EXPECT_EQ(image_3(0, 0), 0);
  EXPECT_EQ(image_3(0, 1), 0);
  EXPECT_EQ(image_3(1, 0), 0);
  EXPECT_EQ(image_3(1, 1), 1);
}

TEST_F(DepthImageTest, CopyToBuffer) {
  MonoImage image_1(2, 2, MemoryType::kUnified);
  image_1(0, 0) = 0;
  image_1(0, 1) = 1;
  image_1(1, 0) = 0;
  image_1(1, 1) = 1;

  uint8_t buffer[4];

  image_1.copyTo(buffer);

  MonoImage image_2(MemoryType::kUnified);
  image_2.copyFrom(2, 2, buffer);

  EXPECT_EQ(image_2(0, 0), 0);
  EXPECT_EQ(image_2(0, 1), 1);
  EXPECT_EQ(image_2(1, 0), 0);
  EXPECT_EQ(image_2(1, 1), 1);
}

TEST_F(DepthImageTest, ResizeLarger) {
  const size_t kRows = rows_ * 2;
  const size_t kCols = cols_ * 2;

  const float* old_data_ptr = depth_frame_.dataPtr();
  depth_frame_.resizeAsync(kRows, kCols, CudaStreamOwning());
  EXPECT_EQ(depth_frame_.rows(), kRows);
  EXPECT_EQ(depth_frame_.cols(), kCols);

  // Expect reallocated buffer when expanding
  EXPECT_NE(depth_frame_.dataPtr(), old_data_ptr);
}

TEST_F(DepthImageTest, ResizeSmaller) {
  const size_t kRows = rows_ / 2;
  const size_t kCols = cols_ / 2;

  const float* old_data_ptr = depth_frame_.dataPtr();
  depth_frame_.resizeAsync(kRows, kCols, CudaStreamOwning());
  EXPECT_EQ(depth_frame_.rows(), kRows);
  EXPECT_EQ(depth_frame_.cols(), kCols);

  // Don't expect reallocated buffer when shrinking
  EXPECT_EQ(depth_frame_.dataPtr(), old_data_ptr);
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
