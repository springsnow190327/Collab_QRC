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
#pragma once

#include <cuda_runtime.h>
#include <stdint.h>
#include <cmath>

#include "nvblox/core/array.h"

namespace nvblox {

/// Color, stored as 8-bit RGB, with helper functions for commonly-used colors.
constexpr int kRgbNumElements = 3;
struct Color : public Array<uint8_t, kRgbNumElements> {
  __host__ __device__ Color() {}
  __host__ __device__ Color(uint8_t r, uint8_t g, uint8_t b)
      : Array<uint8_t, 3>({r, g, b}) {}

  enum Pixel { kRed = 0, kGreen = 1, kBlue = 2 };

  __host__ __device__ uint8_t r() const { return data_[kRed]; }
  __host__ __device__ uint8_t g() const { return data_[kGreen]; }
  __host__ __device__ uint8_t b() const { return data_[kBlue]; }

  __host__ __device__ uint8_t& r() { return data_[kRed]; }
  __host__ __device__ uint8_t& g() { return data_[kGreen]; }
  __host__ __device__ uint8_t& b() { return data_[kBlue]; }

  /// Check if colors are exactly identical.
  __host__ __device__ bool operator==(const Color& other) const {
    return (r() == other.r()) && (g() == other.g()) && (b() == other.b());
  }

  /// Static functions for working with colors
  __host__ __device__ static Color blendTwoColors(const Color& first_color,
                                                  float first_weight,
                                                  const Color& second_color,
                                                  float second_weight);

  // Now a bunch of static colors to use! :)
  __host__ __device__ static const Color White() {
    return Color(255, 255, 255);
  }
  __host__ __device__ static const Color Black() { return Color(0, 0, 0); }
  __host__ __device__ static const Color Gray() { return Color(127, 127, 127); }
  __host__ __device__ static const Color Red() { return Color(255, 0, 0); }
  __host__ __device__ static const Color Green() { return Color(0, 255, 0); }
  __host__ __device__ static const Color Blue() { return Color(0, 0, 255); }
  __host__ __device__ static const Color Yellow() { return Color(255, 255, 0); }
  __host__ __device__ static const Color Orange() { return Color(255, 127, 0); }
  __host__ __device__ static const Color Purple() { return Color(127, 0, 255); }
  __host__ __device__ static const Color Teal() { return Color(0, 255, 255); }
  __host__ __device__ static const Color Pink() { return Color(255, 0, 127); }
};

/// Stream operator for printing Color objects
inline std::ostream& operator<<(std::ostream& os, const Color& color) {
  os << "Color(r=" << static_cast<int>(color.r())
     << ", g=" << static_cast<int>(color.g())
     << ", b=" << static_cast<int>(color.b()) << ")";
  return os;
}

}  // namespace nvblox
