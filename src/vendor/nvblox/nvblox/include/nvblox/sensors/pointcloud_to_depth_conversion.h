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
#pragma once

#include <optional>

#include "nvblox/core/cuda_stream.h"
#include "nvblox/core/time.h"
#include "nvblox/core/types.h"
#include "nvblox/core/unified_vector.h"
#include "nvblox/sensors/camera.h"
#include "nvblox/sensors/image.h"
#include "nvblox/sensors/lidar.h"

namespace nvblox {

/// Convert pointcloud to depth image with optional motion compensation
/// (internal implementation used by the above two functions)
/// @param pointcloud The pointcloud to convert
/// @param T_L_S_scanStart The start transform of the sensor (at timesamp_ms =
/// 0)
/// @param lidar_sensor The lidar sensor
/// @param use_lidar_motion_compensation Whether to use motion compensation.
/// @param maybe_T_L_S_scanEnd The end transform of the sensor (at timesamp_ms =
/// scan_duration_ms)
/// @param maybe_scan_duration_ms The duration of the scan
/// @param depth_image_ptr The depth image to write to
/// @param cuda_stream The CUDA stream to use
template <typename SensorType>
void depthImageFromPointcloudGPU(
    const Pointcloud& pointcloud,                         // NOLINT
    const Transform& T_L_S_scanStart,                     // NOLINT
    const SensorType& lidar_sensor,                       // NOLINT
    const bool use_lidar_motion_compensation,             // NOLINT
    const std::optional<Transform>& maybe_T_L_S_scanEnd,  // NOLINT
    const std::optional<Time>& maybe_scan_duration_ms,    // NOLINT
    DepthImage* depth_image_ptr,                          // NOLINT
    const CudaStream& cuda_stream);

}  // namespace nvblox
