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
#pragma once

#include <torch/script.h>

#include <ATen/ATen.h>
#include <torch/custom_class.h>

#include <nvblox/core/indexing.h>
#include <nvblox/mapper/mapper.h>
#include <nvblox/utils/timing.h>
#include <nvblox/gpu_hash/internal/cuda/gpu_indexing.cuh>
#include <vector>
#include "nvblox/core/types.h"
#include "nvblox/integrators/weighting_function.h"
#include "nvblox/io/mesh_io.h"
#include "nvblox/io/ply_writer.h"
#include "nvblox/map/layer.h"
#include "nvblox/map/voxels.h"
#include "nvblox/mesh/mesh.h"
#include "nvblox/rays/sphere_tracer.h"

#include "nvblox_torch/convert_tensors.h"
#include "nvblox_torch/py_layer.h"
#include "nvblox_torch/py_mapper_params.h"
#include "nvblox_torch/py_mesh.h"
#include "nvblox_torch/py_sensor.h"

namespace pynvblox {

struct Mapper : torch::CustomClassHolder {
  Mapper(std::vector<double> voxel_size_m,
         std::vector<std::string> projective_layer_type,
         c10::intrusive_ptr<MapperParams> mapper_params);

  ~Mapper() = default;

  // Sensor-based integration (unified entry point)
  void integrateDepth(torch::Tensor depth_frame_t, torch::Tensor T_L_C_t,
                      c10::intrusive_ptr<PySensor> sensor,
                      std::optional<torch::Tensor> mask_frame_t = std::nullopt,
                      long mapper_id = -1);

  void integrateColor(torch::Tensor color_frame_t, torch::Tensor T_L_C_t,
                      c10::intrusive_ptr<PySensor> sensor,
                      std::optional<torch::Tensor> mask_frame_t = std::nullopt,
                      long mapper_id = -1);

  void integrateFeatures(
      torch::Tensor feature_frame_t, torch::Tensor T_L_C_t,
      c10::intrusive_ptr<PySensor> sensor,
      std::optional<torch::Tensor> mask_frame_t = std::nullopt,
      long mapper_id = -1);

  void updateEsdf(long mapper_id = -1);

  void updateColorMesh(long mapper_id = -1);

  void updateFeatureMesh(long mapper_id = -1);

  // Params
  c10::intrusive_ptr<MapperParams> getMapperParams();

  /// @brief Copies the mesh layer to a single monolith mesh on the CPU.
  /// @return A nvblox Mesh on the CPU.
  c10::intrusive_ptr<pynvblox::PyColorMesh> getColorMesh(long mapper_id = 0);
  c10::intrusive_ptr<pynvblox::PyFeatureMesh> getFeatureMesh(
      long mapper_id = 0);

  void fullUpdate(torch::Tensor depth_frame_t, torch::Tensor color_frame_t,
                  torch::Tensor T_L_C_t, torch::Tensor intrinsics_t,
                  long mapper_id);

  void decayTsdf(long mapper_id = -1);
  void decayOccupancy(long mapper_id = -1);

  void clear(long mapper_id = -1);

  void addMapper(double voxel_size_m, std::string projective_layer_type,
                 const MapperParams& mapper_params);

  long getNumMappers() const;

  std::shared_ptr<nvblox::Mapper> getNvbloxMapper(long mapper_id);

  c10::intrusive_ptr<PyTsdfLayer> tsdf_layer(long mapper_id = 0);
  c10::intrusive_ptr<PyColorLayer> color_layer(long mapper_id = 0);
  c10::intrusive_ptr<PyFeatureLayer> feature_layer(long mapper_id = 0);

  torch::Tensor renderDepthImage(torch::Tensor camera_pose,
                                 torch::Tensor intrinsics, int64_t img_height,
                                 int64_t img_width, double max_ray_length,
                                 int64_t max_steps, long mapper_id);

  std::vector<torch::Tensor> renderDepthAndColorImage(
      torch::Tensor camera_pose, torch::Tensor intrinsics, int64_t img_height,
      int64_t img_width, double max_ray_length, int64_t max_steps,
      long mapper_id);

  /// @brief Queries the ESDF at a set of locations.
  /// @param[out] output_tensor Output tensor. Nx4 tensor containing
  /// [x,y,z,distance] for each query point where x,y,z is a vector from the
  /// query point to the closest surface voxel.
  /// @param query_sphere Query. Nx4 tensor [x,y,z,radius] for each query point.
  /// Radius is subtracted from the ESDF distance.
  /// @param mapper_id The ID of the mapper containing the map to query.
  /// @return A Nx1 tensor containing the distances.
  torch::Tensor queryEsdf(torch::Tensor output_tensor,
                          const torch::Tensor query_sphere, long mapper_id);
  torch::Tensor queryMultiEsdf(torch::Tensor output_tensor,
                               const torch::Tensor query_sphere);

  /// @brief Query the feature layer.
  /// @param query_positions An Nx3 tensor containing the [x,y,z] positions of
  /// the query locations.
  /// @param mapper_id The ID of the mapper containing the map to query.
  /// @param output_tensor Nx(F+1) output tensor containing the feature values
  /// for each of the N query positions. The last element of the tensor contains
  /// the feature weights.
  /// @return A NxF tensor containing the feaures.
  torch::Tensor queryFeatures(torch::Tensor output_tensor,
                              const torch::Tensor query_positions,
                              long mapper_id);

  /// @brief Query the TSDF layer.
  /// @param output_tensor Nx2 output tensor containing the TSDF value and
  /// weight for each query position.
  /// @param query_positions An Nx3 tensor containing the [x,y,z] positions of
  /// the query locations.
  /// @param mapper_id The ID of the mapper containing the map to query.
  /// @return A Nx2 tensor containing the TSDF values and weights.
  torch::Tensor queryTsdf(torch::Tensor output_tensor,
                          const torch::Tensor query_positions, long mapper_id);
  torch::Tensor queryMultiTsdf(torch::Tensor output_tensor,
                               const torch::Tensor query_positions);

  // TODO(alexmillane, 2024.10.28): Still to be cleaned up and tested.
  torch::Tensor queryMultiOccupancy(torch::Tensor outputs,
                                    const torch::Tensor query_positions);

  bool outputColorMeshPly(std::string mesh_output_path, long mapper_id = 0);
  bool outputBloxMap(std::string blox_output_path, long mapper_id = 0);

  void loadFromFile(std::string file_path, long mapper_id = 0);

  c10::intrusive_ptr<Mapper> clone() const {
    return c10::make_intrusive<Mapper>(voxel_size_m_, projective_layer_type_,
                                       mapper_params_);
  }
  std::string printTiming() const;

 protected:
  /// This function gets a layer as a managed pointer such that it can be
  /// returned to python.
  template <typename PyLayerType>
  c10::intrusive_ptr<PyLayerType> get_layer(
      const long mapper_id, const std::string& name,
      nvblox::ProjectiveLayerType required_projective_layer_type =
          nvblox::ProjectiveLayerType::kNone);

  /// Transfer the specified layer's hashes to the host and device buffers.
  /// Layer specified by the template parameter.
  template <typename BlockType>
  void transferGPUHashesAsync(
      nvblox::host_vector<nvblox::Index3DDeviceHashMapType<BlockType>>*
          hash_transfer_buffer_host,
      nvblox::device_vector<nvblox::Index3DDeviceHashMapType<BlockType>>*
          hash_transfer_buffer_device,
      const nvblox::CudaStream& stream);

  /// A list of mappers.
  std::vector<std::shared_ptr<nvblox::Mapper>> mappers_;

  /// The voxel size for each mapper.
  std::vector<double> voxel_size_m_;

  /// The mapper parameters for each mapper.
  c10::intrusive_ptr<MapperParams> mapper_params_;

  /// The block sizes for each mapper.
  nvblox::device_vector<float> block_sizes_m_gpu_;

  /// The type of projective layer for each mapper.
  std::vector<std::string> projective_layer_type_;

  /// Staging buffers for transferring the hashes to the GPU.
  using EsdfGPUHash = nvblox::Index3DDeviceHashMapType<nvblox::EsdfBlock>;
  nvblox::host_vector<EsdfGPUHash> esdf_hash_transfer_buffer_host_;
  nvblox::device_vector<EsdfGPUHash> esdf_hash_transfer_buffer_device_;
  using TsdfGPUHash = nvblox::Index3DDeviceHashMapType<nvblox::TsdfBlock>;
  nvblox::host_vector<TsdfGPUHash> tsdf_hash_transfer_buffer_host_;
  nvblox::device_vector<TsdfGPUHash> tsdf_hash_transfer_buffer_device_;
  using OccupancyGPUHash =
      nvblox::Index3DDeviceHashMapType<nvblox::OccupancyBlock>;
  nvblox::host_vector<OccupancyGPUHash> occupancy_hash_transfer_buffer_host_;
  nvblox::device_vector<OccupancyGPUHash>
      occupancy_hash_transfer_buffer_device_;
};

}  // namespace pynvblox
