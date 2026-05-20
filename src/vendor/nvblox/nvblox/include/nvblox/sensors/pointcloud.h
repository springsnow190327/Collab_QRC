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

constexpr MemoryType kDefaultPointcloudMemoryType = MemoryType::kDevice;

/// Pointcloud that lives in either device, host or unified memory.
/// We represent a pointcloud as a vector of 3D vectors and optionally a vector
/// of per-point timestamps (relative to the scan start).
///
/// NOTE: If timestamps exist, they must have the same size as points (per-point
/// timestamps). This is enforced by:
/// - resize/reserve operations that synchronize both vectors
/// - the timestamp accessors which validate on access (with
/// checkTimestampsConsistency)
class Pointcloud {
 public:
  /// Construct a pointcloud with a given number of points (and optionally
  /// per-point timestamps)
  Pointcloud(int size, MemoryType memory_type = kDefaultPointcloudMemoryType,
             bool init_timestamps = false);
  /// Construct an empty pointcloud
  Pointcloud(MemoryType memory_type = kDefaultPointcloudMemoryType,
             bool init_timestamps = false);

  /// Move operations
  Pointcloud(Pointcloud&& other) = default;
  Pointcloud& operator=(Pointcloud&& other) = default;
  Pointcloud(const Pointcloud& other) = delete;

  /// Copy from another pointcloud
  void copyFromAsync(const Pointcloud& other, const CudaStream& cuda_stream);

  /// Copy points from a vector
  void copyPointsFromAsync(const std::vector<Vector3f>& points,
                           const CudaStream& cuda_stream);
  void copyPointsFromAsync(const unified_vector<Vector3f>& points,
                           const CudaStream& cuda_stream);

  /// Copy timestamps from a vector
  void copyTimestampsFromAsync(const unified_vector<Time>& timestamps_ms,
                               const CudaStream& cuda_stream);
  void copyTimestampsFromAsync(const std::vector<Time>& timestamps_ms,
                               const CudaStream& cuda_stream);

  /// Deep copy constructor
  Pointcloud(const Pointcloud& other, MemoryType memory_type);
  Pointcloud& operator=(const Pointcloud& other);

  /// Expand memory available
  void resizeAsync(int size, const CudaStream& cuda_stream);
  void resize(int size);
  void reserveAsync(int size, const CudaStream& cuda_stream);
  void reserve(int size);

  /// Attributes
  int size() const { return points_.size(); }
  MemoryType memory_type() const { return points_.memory_type(); }
  bool empty() const { return points_.empty(); }

  /// Points access
  const Vector3f& point(int index) const { return points_[index]; }
  Vector3f& point(int index) { return points_[index]; }
  const unified_vector<Vector3f>& points() const { return points_; }
  unified_vector<Vector3f>& points() { return points_; }

  /// Timestamps access
  const std::optional<unified_vector<Time>>& timestamps_ms() const;
  std::optional<unified_vector<Time>>& timestamps_ms();

  /// Points raw pointer access
  Vector3f* pointsPtr() { return points_.data(); }
  const Vector3f* pointsConstPtr() const { return points_.data(); }

  /// Timestamps raw pointer access
  Time* timestampsPtr();
  const Time* timestampsConstPtr() const;

  /// Add a point
  void push_back(Vector3f&& point) { points_.push_back(point); }

 protected:
  /// Helper function to check that timestamps are consistent with points
  void checkTimestampsConsistency() const;

  /// Points in the pointcloud
  unified_vector<Vector3f> points_;
  /// Per-point timestamps in the pointcloud (optional)
  /// Expected to be relative to the scan start.
  std::optional<unified_vector<Time>> timestamps_ms_;
};

/// Transforms the points in a pointcloud into another frame
/// @param T_out_in Transform that takes a point in frame "in" to frame "out".
/// @param pointcloud_in Pointcloud in frame "in".
/// @param[out] pointcloud_out Pointer to pointcloud in frame "out".
void transformPointcloudOnGPU(const Transform& T_out_in,        // NOLINT
                              const Pointcloud& pointcloud_in,  // NOLINT
                              Pointcloud* pointcloud_out_ptr);

/// Transforms the points in a pointcloud into another frame
/// See transformPointcloudOnGPU(). Same function just on a stream.
void transformPointcloudOnGPU(const Transform& T_out_in,        // NOLINT
                              const Pointcloud& pointcloud_in,  // NOLINT
                              Pointcloud* pointcloud_out_ptr,   // NOLINT
                              CudaStream* cuda_stream_ptr);

}  // namespace nvblox
