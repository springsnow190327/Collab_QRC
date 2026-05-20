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

// Forward declarations of helper kernels (implemented in
// pointcloud_to_depth_conversion_.cu)
__global__ void initDepthImageKernel(const int rows, const int cols,
                                     const float init_value,
                                     float* depth_image);

__global__ void setSentinelDepthToZeroKernel(const int rows, const int cols,
                                             const float sentinel_value,
                                             float* depth_image);

__device__ inline Vector3f lerpTranslation(const Vector3f& t1,
                                           const Vector3f& t2, float alpha) {
  // Linear interpolation of translation
  return (1.0f - alpha) * t1 + alpha * t2;
}

template <typename SensorType>
__global__ void depthImageFromPointcloudKernel(
    const Vector3f* points,                    // NOLINT
    const Time* point_timestamps_ms,           // NOLINT
    const SensorType lidar_sensor,             // NOLINT
    const Eigen::Quaternionf q_L_S_scanStart,  // NOLINT
    const Vector3f t_L_S_scanStart,            // NOLINT
    const Eigen::Quaternionf q_L_S_scanEnd,    // NOLINT
    const Vector3f t_L_S_scanEnd,              // NOLINT
    const Time scan_duration_ms,               // NOLINT
    const int size,                            // NOLINT
    const bool apply_motion_compensation,      // NOLINT
    float* depth_image) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (idx >= size) {
    return;
  }

  // Read a point from global memory
  Vector3f point = points[idx];

  if (isnan(point.x()) || isnan(point.y()) || isnan(point.z())) {
    return;
  }

  if (apply_motion_compensation) {
    // Apply motion compensation:
    // - In general, we assume all points lie in the same sensor frame
    //   (T_L_S_scanStart) at scan start.
    // - This becomes inaccurate if the sensor moves significantly
    //   during the scan.
    // - To correct this, we interpolate the sensor pose between scan start and
    //   scan end.
    // - The interpolated pose approximates the sensor frame at each point's
    //   acquisition time.
    // - For downstream processing, we still want all points
    //   in the common frame T_L_S_scanStart.
    // - Therefore, we use the interpolated pose to map each
    //   point to T_L_S_scanStart.

    // Read the point's relative timestamp
    const Time point_timestamp_ms = point_timestamps_ms[idx];

    // Notes:
    // - point_timestamp_ms is relative to scan start and lies in [0,
    // scan_duration_ms].
    // - At point_timestamp_ms = 0, the sensor frame is T_L_S_scanStart.
    // - At point_timestamp_ms = scan_duration_ms, the sensor frame is
    // T_L_S_scanEnd.
    NVBLOX_CHECK(scan_duration_ms > Time(0), "Scan duration must be positive");
    NVBLOX_CHECK(point_timestamp_ms >= Time(0),
                 "Relative point timestamp must be non-negative");
    NVBLOX_CHECK(
        point_timestamp_ms <= scan_duration_ms,
        "Relative point timestamp must be less than or equal to scan duration");

    // The interpolation factor (alpha) is the ratio
    // of the point's relative timestamp to the scan duration.
    // NOTE: Converting Time -> int64_t -> float for calculating alpha.
    const float alpha =
        static_cast<float>(static_cast<int64_t>(point_timestamp_ms)) /
        static_cast<float>(static_cast<int64_t>(scan_duration_ms));
    NVBLOX_CHECK(alpha >= 0.0f && alpha <= 1.0f,
                 "Interpolation factor must be between 0 and 1");

    // Interpolate the sensor transform between scan start and scan end time.
    // This approximates the sensor frame at the time of the point acquisition.
    const Eigen::Quaternionf q_L_S_pointAcquisition =
        q_L_S_scanStart.slerp(alpha, q_L_S_scanEnd);
    const Vector3f t_L_S_pointAcquisition =
        lerpTranslation(t_L_S_scanStart, t_L_S_scanEnd, alpha);

    // The point is in the sensor frame at the time of the point acquisition.
    // Transforming it to the global frame: point_L = T_L_S_pointAcquisition *
    // point
    const Vector3f point_L =
        q_L_S_pointAcquisition * point + t_L_S_pointAcquisition;

    // Transforming the point back to the sensor frame at scan start.
    // This puts all points in a common frame (T_L_S_scanStart) for later
    // processing: point = T_L_S_scanStart^-1 * point_L
    point = q_L_S_scanStart.inverse() * (point_L - t_L_S_scanStart);
  }

  // Project
  Vector2f u_C;
  if (!lidar_sensor.project(point, &u_C)) {
    return;
  }

  // Write the depth to the image
  // NOTE: Multiple points can project to the same pixel. We use atomic min
  // to ensure we end up with the minimum depth projection.
  // For this to work, the depth image must be initialized
  // to a value above the maximum depth.
  const float depth = lidar_sensor.getDepth(point);
  atomicMinFloat(
      &image::access(u_C.y(), u_C.x(), lidar_sensor.cols(), depth_image),
      depth);
}

template <typename SensorType>
void depthImageFromPointcloudGPU(
    const Pointcloud& pointcloud,                         // NOLINT
    const Transform& T_L_S_scanStart,                     // NOLINT
    const SensorType& lidar_sensor,                       // NOLINT
    const bool use_lidar_motion_compensation,             // NOLINT
    const std::optional<Transform>& maybe_T_L_S_scanEnd,  // NOLINT
    const std::optional<Time>& maybe_scan_duration_ms,    // NOLINT
    DepthImage* depth_image_ptr,                          // NOLINT
    const CudaStream& cuda_stream) {
  timing::Timer timer("pointcloud/depth_image_from_pointcloud");
  CHECK(lidar_sensor.sensor_modality() == SensorModality::kLidar)
      << "Pointcloud to depth image conversion is only intended for lidar "
         "sensors";
  CHECK(pointcloud.memory_type() == MemoryType::kDevice ||
        pointcloud.memory_type() == MemoryType::kUnified);
  CHECK(depth_image_ptr->memory_type() == MemoryType::kDevice);

  // Extract quaternion and translation from the scan start transform.
  const Eigen::Quaternionf q_L_S_scanStart(T_L_S_scanStart.rotation());
  const Vector3f t_L_S_scanStart = T_L_S_scanStart.translation();

  Eigen::Quaternionf q_L_S_scanEnd;
  Vector3f t_L_S_scanEnd;
  Time scan_duration_ms;
  if (use_lidar_motion_compensation) {
    // Check if we have the necessary data for motion compensation:
    // - Valid scan duration (time difference between start and end transforms)
    // - Per-point timestamps (relative to scan start)
    CHECK(pointcloud.timestamps_ms().has_value());
    CHECK(maybe_T_L_S_scanEnd.has_value());
    CHECK(maybe_scan_duration_ms.has_value());
    CHECK(maybe_scan_duration_ms.value() > Time(0));

    // If motion compensation is enabled,
    // we extract the lidar scan data from the optionals.
    q_L_S_scanEnd = Eigen::Quaternionf(maybe_T_L_S_scanEnd->rotation());
    t_L_S_scanEnd = maybe_T_L_S_scanEnd->translation();
    scan_duration_ms = maybe_scan_duration_ms.value();
  }

  // Resize the image if required.
  depth_image_ptr->resizeAsync(lidar_sensor.rows(), lidar_sensor.cols(),
                               cuda_stream);

  // Initialize the entire image to a sentinel value (max float).
  // This is needed to set each pixel to the minimum depth projection
  // using atomicMinFloat in depthImageFromPointcloudKernel.
  const dim3 kThreadsPerBlock2D(16, 16);
  const dim3 num_blocks_2d(
      divideRoundUp(depth_image_ptr->cols(), kThreadsPerBlock2D.x),
      divideRoundUp(depth_image_ptr->rows(), kThreadsPerBlock2D.y));
  constexpr float kSentinelValue = 1e6f;
  initDepthImageKernel<<<num_blocks_2d, kThreadsPerBlock2D, 0, cuda_stream>>>(
      depth_image_ptr->rows(), depth_image_ptr->cols(), kSentinelValue,
      depth_image_ptr->dataPtr());
  checkCudaErrors(cudaPeekAtLastError());

  // Convert the pointcloud to a depth image on the GPU.
  constexpr int kNumThreadsPerBlock = 256;
  int num_blocks = divideRoundUp(pointcloud.size(), kNumThreadsPerBlock);
  depthImageFromPointcloudKernel<<<num_blocks, kNumThreadsPerBlock, 0,
                                   cuda_stream>>>(
      pointcloud.pointsConstPtr(), pointcloud.timestampsConstPtr(),
      lidar_sensor, q_L_S_scanStart, t_L_S_scanStart, q_L_S_scanEnd,
      t_L_S_scanEnd, scan_duration_ms, pointcloud.size(),
      use_lidar_motion_compensation, depth_image_ptr->dataPtr());
  checkCudaErrors(cudaPeekAtLastError());

  // Cleanup: Set remaining sentinel values (max float) to 0.
  // These are pixels with no valid depth projection.
  setSentinelDepthToZeroKernel<<<num_blocks_2d, kThreadsPerBlock2D, 0,
                                 cuda_stream>>>(
      depth_image_ptr->rows(), depth_image_ptr->cols(), kSentinelValue,
      depth_image_ptr->dataPtr());
  checkCudaErrors(cudaPeekAtLastError());
  checkCudaErrors(cudaStreamSynchronize(cuda_stream));
}

}  // namespace nvblox
