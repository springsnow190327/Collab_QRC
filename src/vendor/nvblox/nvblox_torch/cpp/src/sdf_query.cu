/*
 * Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */
#include "nvblox_torch/sdf_query.cuh"

#include <c10/cuda/CUDAStream.h>
#include <iostream>

#include "nvblox/map/voxels.h"

#include "nvblox/core/feature_array.h"

namespace pynvblox {
namespace sdf {

// Store pointers used for ESDF extraction
class ExtractEsdfPointers {
  static constexpr int kStride = 4;

 public:
  __device__ ExtractEsdfPointers(const int query_index,
                                 const bool extract_gradients,
                                 const float* query_spheres, float* out_tensor)
      : query_location(&query_spheres[kStride * query_index]),
        radius_in(query_spheres + query_index * kStride + 3) {
    if (extract_gradients) {
      // Strided output buffer if we're extracting gradients
      distance_out = out_tensor + query_index * kStride + 3;
      gradient_out = out_tensor + kStride * query_index;
    } else {
      // Non-strided output buffer if we only extract distances
      distance_out = out_tensor + query_index;
      gradient_out = nullptr;
    }
  }

  const Eigen::Map<const nvblox::Vector3f> query_location;
  const float* radius_in = nullptr;
  float* distance_out = nullptr;
  float* gradient_out = nullptr;
};

// Extract data from an ESDF voxel.
//
// I/O stored in the ExtractEsdfPointers object:
//   .radius_in: radius of the query sphere
//   .distance_out store the extracted distance
//   .gradient_out (if non-null) stores the gradient
//
// min_distance_over_all_mappers_opt is used for skipping extraction in the case
// of multiple queries. It tracks the minimum distance observed over multiple
// queries. Hence, For non-minimum queries:
//  * gradient extraction is skipped/undefined
//  * extracted distance is set to min_distance_over_all_mappers_opt
//  * function returns false.
__device__ bool extractEsdf(const nvblox::EsdfVoxel* esdf_voxel,
                            const float voxel_size, ExtractEsdfPointers& ptrs,
                            float* min_distance_over_all_mappers_opt) {
  NVBLOX_CHECK(esdf_voxel != nullptr, "Invalid input");
  NVBLOX_CHECK(ptrs.distance_out != nullptr, "Invalid input");

  // Early exit if unobserved
  if (!esdf_voxel->observed) {
    *(ptrs.distance_out) = kESDFUnknownDistance;
    return false;
  }

  // Get the distance of the relevant voxel.
  float distance = voxel_size * sqrt(esdf_voxel->squared_distance_vox);
  // If it's inside, we set the value to be negative
  if (esdf_voxel->is_inside) {
    distance = -distance;
  }

  // Subtract the radius of the query sphere.
  // TODO(nvblox_torch_refactor): Move this to another function.
  const float sphere_distance = distance - *(ptrs.radius_in);

  // Handle optionally provided min distance
  if (min_distance_over_all_mappers_opt != nullptr)
    // Early exit if the distance is larger than the min value obsered so far
    if (sphere_distance > *min_distance_over_all_mappers_opt) {
      *(ptrs.distance_out) = *min_distance_over_all_mappers_opt;
      return false;
    } else {
      // Update min observed value
      *min_distance_over_all_mappers_opt = sphere_distance;
    }

  // Set the output distance
  *(ptrs.distance_out) = sphere_distance;

  // Optionally extract the gradient
  if (ptrs.gradient_out != nullptr) {
    Eigen::Map<Eigen::Vector3f> gradient_vector(ptrs.gradient_out);

    // Gradient vector
    // NOTE(alexmillane): If the distance is zero, we set the gradient to zero.
    constexpr float kEpsilon = 1e-6;
    if (distance > kEpsilon) {
      gradient_vector =
          (-voxel_size / distance) * esdf_voxel->parent_direction.cast<float>();
    } else {
      gradient_vector = nvblox::Vector3f::Zero();
    }
  }
  return true;
}

__device__ bool extractTsdf(const nvblox::TsdfVoxel* tsdf_voxel,
                            float* distance_ptr, float* weight_ptr) {
  *distance_ptr = tsdf_voxel->distance;
  *weight_ptr = tsdf_voxel->weight;
  return true;
}

__device__ bool extractFeatureAndWeight(
    const nvblox::FeatureVoxel* feature_voxel,
    nvblox::FeatureArray* feature_ptr, float* weight_ptr) {
  *feature_ptr = feature_voxel->feature;
  *weight_ptr = feature_voxel->weight;
  return true;
}

template <typename VoxelType, typename OnQueryFunction>
__device__ bool query(
    nvblox::Index3DDeviceHashMapType<nvblox::VoxelBlock<VoxelType>> block_hash,
    const nvblox::Vector3f& point, const float block_size,
    OnQueryFunction on_query_function) {
  // Block to voxel size.
  const float voxel_size =
      block_size / nvblox::VoxelBlock<VoxelType>::kVoxelsPerSide;
  // Get the correct block from the hash.
  VoxelType* voxel_ptr;
  if (nvblox::getVoxelAtPosition<VoxelType>(block_hash, point, block_size,
                                            &voxel_ptr)) {
    const bool success = on_query_function(voxel_ptr, voxel_size);
    return success;
  }
  return false;
}

// Calling conventions
// - Query spheres: [x, y, z, r]
//     The center and radius of the query sphere.
// - out_tensor: [vx, vy, vz, distance]
//     Combination of the (optional) surface direction and distance.
__global__ void queryESDFKernel(
    int64_t num_queries, bool extract_gradients,
    nvblox::Index3DDeviceHashMapType<nvblox::EsdfBlock> block_hash,
    float block_size, const float* query_spheres, float* out_tensor) {
  // Figure out which point this thread should be querying.
  const size_t query_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_index >= num_queries) {
    return;
  }

  ExtractEsdfPointers ptrs(query_index, extract_gradients, query_spheres,
                           out_tensor);

  query<nvblox::EsdfVoxel>(
      block_hash, ptrs.query_location, block_size,
      [&](nvblox::EsdfVoxel* esdf_voxel, const float voxel_size) -> bool {
        return extractEsdf(esdf_voxel, voxel_size, ptrs, nullptr);
      });
}

__global__ void queryESDFMultiMapperKernel(
    int64_t num_mappers, int64_t num_queries, bool extract_gradients,
    nvblox::Index3DDeviceHashMapType<nvblox::EsdfBlock>* hashes,
    float* block_sizes, const float* query_spheres, float* out_tensor) {
  // Figure out which point this thread should be querying.
  const size_t query_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_index >= num_queries) {
    return;
  }

  ExtractEsdfPointers ptrs(query_index, extract_gradients, query_spheres,
                           out_tensor);

  // Stores the minimum distance encountered
  float min_distance_over_all_mappers = kMaxDistance;

  // Loop over the maps
  for (int i = 0; i < num_mappers; i++) {
    // Extract this map
    const float block_size = block_sizes[i];
    auto block_hash = hashes[i];

    // Query
    query<nvblox::EsdfVoxel>(
        block_hash, ptrs.query_location, block_size,
        [&](nvblox::EsdfVoxel* esdf_voxel, const float voxel_size) -> bool {
          return extractEsdf(esdf_voxel, voxel_size, ptrs,
                             &min_distance_over_all_mappers);
        });
  }
}
__global__ void queryFeatureKernel(
    int64_t num_queries,
    nvblox::Index3DDeviceHashMapType<nvblox::FeatureBlock> block_hash,
    float block_size, const float* query_positions, at::Half* output_tensor) {
  // Figure out which point this thread should be querying.
  const size_t query_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_index >= num_queries) {
    return;
  }

  const int kQueryPositionNumElements = 3;
  Eigen::Map<const Eigen::Vector3f> query_location(
      &query_positions[query_index * kQueryPositionNumElements]);

  nvblox::FeatureArray feature;
  float weight = 0.0;
  const bool query_success = query<nvblox::FeatureVoxel>(
      block_hash, query_location, block_size,
      [&](nvblox::FeatureVoxel* feature_voxel, const float voxel_size) -> bool {
        return extractFeatureAndWeight(feature_voxel, &feature, &weight);
      });

  if (query_success) {
    constexpr size_t kFeatureNumElements = nvblox::FeatureArray::size();
    constexpr size_t kOutputNumElements = kFeatureNumElements + 1;
    // TODO(dtingdahl) Add a memcpy function to Array to avoid for loop
    for (int i = 0; i < kFeatureNumElements; ++i) {
      output_tensor[query_index * kOutputNumElements + i] = feature[i];
    }
    output_tensor[query_index * kOutputNumElements + kFeatureNumElements] =
        weight;
  }
}

__global__ void queryTSDFKernel(
    int64_t num_queries,
    nvblox::Index3DDeviceHashMapType<nvblox::TsdfBlock> block_hash,
    float block_size, const float* query_spheres, float* output_tensor) {
  // Figure out which point this thread should be querying.
  const size_t query_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_index >= num_queries) {
    return;
  }

  // Map the input and outputs
  Eigen::Map<const Eigen::Vector3f> query_location(
      &query_spheres[query_index * 3]);
  float* out_tsdf_ptr = &output_tensor[query_index * 2];
  float* out_weight_ptr = &output_tensor[query_index * 2 + 1];

  // Query
  float distance = -kMaxDistance;
  float weight = 0.f;
  const bool query_success = query<nvblox::TsdfVoxel>(
      block_hash, query_location, block_size,
      [&](nvblox::TsdfVoxel* tsdf_voxel, const float voxel_size) -> bool {
        return extractTsdf(tsdf_voxel, &distance, &weight);
      });

  // Write to global memory
  if (query_success) {
    *out_tsdf_ptr = distance;
    *out_weight_ptr = weight;
  }
}

__global__ void queryTSDFMultiMapperKernel(
    int64_t num_mappers, int64_t num_queries,
    nvblox::Index3DDeviceHashMapType<nvblox::TsdfBlock>* hashes,
    float* block_sizes, const float* query_spheres, float* output_tensor) {
  // Figure out which point this thread should be querying.
  size_t query_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_index >= num_queries) {
    return;
  }

  // Map the input and outputs
  Eigen::Map<const Eigen::Vector3f> query_location(
      &query_spheres[query_index * 3]);
  float* out_tsdf_ptr = &output_tensor[query_index * 2];
  float* out_weight_ptr = &output_tensor[query_index * 2 + 1];

  // Loop over mappers querying and storing the minimum.
  float min_distance = kMaxDistance;
  float weight_at_min = 0;
  for (int i = 0; i < num_mappers; i++) {
    // Extract this map
    const float block_size = block_sizes[i];
    auto block_hash = hashes[i];

    // Query
    float distance = -kMaxDistance;
    float weight = 0.f;
    const bool query_success = query<nvblox::TsdfVoxel>(
        block_hash, query_location, block_size,
        [&](nvblox::TsdfVoxel* tsdf_voxel, const float voxel_size) -> bool {
          return extractTsdf(tsdf_voxel, &distance, &weight);
        });

    // Save if minimum absolute distance.
    if (query_success) {
      if (distance < min_distance) {
        // min_abs_distance = abs_distance;
        min_distance = distance;
        weight_at_min = weight;
      }
    }
  }

  // Write to global memory
  *out_tsdf_ptr = min_distance;
  *out_weight_ptr = weight_at_min;
}

__global__ void queryOccupancyMultiMapperKernel(
    int64_t num_mappers, int64_t num_queries,
    nvblox::Index3DDeviceHashMapType<nvblox::OccupancyBlock>* hashes,
    float* block_sizes, const float* query_positions, float* out_log_odds) {
  // Figure out which point this thread should be querying.
  size_t query_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (query_index >= num_queries) {
    return;
  }

  // read data into vector3f:
  nvblox::Vector3f query_location;
  query_location(0) = query_positions[query_index * 3 + 0];
  query_location(1) = query_positions[query_index * 3 + 1];
  query_location(2) = query_positions[query_index * 3 + 2];

  float max_log_odds = nvblox::logOddsFromProbability(0);

  for (int i = 0; i < num_mappers; i++) {
    const float block_size = block_sizes[i];

    // Get the correct block from the hash.
    nvblox::OccupancyVoxel* occupancy_voxel;
    if (nvblox::getVoxelAtPosition<nvblox::OccupancyVoxel>(
            hashes[i], query_location, block_size, &occupancy_voxel)) {
      if (occupancy_voxel->log_odds > max_log_odds) {
        max_log_odds = occupancy_voxel->log_odds;
      }
    }
  }
  out_log_odds[query_index] = max_log_odds;
}

}  // namespace sdf
}  // namespace pynvblox
