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
#include "nvblox_torch/py_scene.h"

#include <nvblox/primitives/scene.h>

namespace pynvblox {

std::vector<std::string> Scene::getPrimitiveTypesList() const {
  std::vector<std::string> types;
  for (auto primitive_type : scene_->getPrimitiveTypeList()) {
    types.push_back(nvblox::primitives::Primitive::toString(primitive_type));
  }
  return types;
}

void Scene::setAABB(std::vector<double> low, std::vector<double> high) {
  // TODO: improve the error checking, add exceptions etc
  if (low.size() != 3 || high.size() != 3) {
    std::cerr << "Scene::setAABB expects two vectors of length 3. Ignoring "
                 "invalid request to setAABB."
              << std::endl;
    return;
  }
  scene_->aabb() = nvblox::AxisAlignedBoundingBox(
      nvblox::Vector3f(low[0], low[1], low[2]),
      nvblox::Vector3f(high[0], high[1], high[2]));
}

std::tuple<std::vector<double>, std::vector<double>> Scene::getAABB() {
  const auto min = scene_->aabb().min();
  const auto max = scene_->aabb().max();
  return {{min.x(), min.y(), min.z()}, {max.x(), max.y(), max.z()}};
}

void Scene::addPlaneBoundaries(double x_min, double x_max, double y_min,
                               double y_max) {
  scene_->addPlaneBoundaries(x_min, x_max, y_min, y_max);
}

void Scene::addGroundLevel(double level) { scene_->addGroundLevel(level); }

void Scene::addCeiling(double ceiling) { scene_->addCeiling(ceiling); }

void Scene::addPrimitive(std::string type, std::vector<double> prim_params) {
  // Cube
  if (type == "cube") {
    if (prim_params.size() != 6) {
      std::cerr << "Scene::addPrimitive excepts 6 parameters for type 'cube'. "
                   "Ignoring invalid request to addPrimitive"
                << std::endl;
      return;
    }
    scene_->addPrimitive(std::make_unique<nvblox::primitives::Cube>(
        nvblox::Vector3f(prim_params[0], prim_params[1], prim_params[2]),
        nvblox::Vector3f(prim_params[3], prim_params[4], prim_params[5])));
    // Sphere
  } else if (type == "sphere") {
    if (prim_params.size() != 4) {
      std::cerr << "Scene::addPrimitive excepts 4 parameters for type "
                   "'sphere'. Ignoring invalid request to addPrimitive"
                << std::endl;
      return;
    }
    scene_->addPrimitive(std::make_unique<nvblox::primitives::Sphere>(
        nvblox::Vector3f(prim_params[0], prim_params[1], prim_params[2]),
        prim_params[3]));
    // Plane
  } else if (type == "plane") {
    if (prim_params.size() != 6) {
      std::cerr << "Scene::addPrimitive excepts 6 parameters for type "
                   "'plane'. Ignoring invalid request to addPrimitive"
                << std::endl;
      return;
    }
    scene_->addPrimitive(std::make_unique<nvblox::primitives::Plane>(
        nvblox::Vector3f(prim_params[0], prim_params[1], prim_params[2]),
        nvblox::Vector3f(prim_params[3], prim_params[4], prim_params[5])));
  } else {
    std::cerr << "Scene::addPrimitive received invalid primitive type: " << type
              << std::endl;
    return;
  }
}

void Scene::toMapper(c10::intrusive_ptr<Mapper> mapper, long mapper_id) {
  // Which mappers ids do we modify?
  std::pair start_end_id = {0, mapper->getNumMappers()};
  if (mapper_id >= 0) {
    start_end_id.first = mapper_id;
    start_end_id.second = mapper_id + 1;
  }
  CHECK_GE(start_end_id.first, 0);
  CHECK_LE(start_end_id.second, mapper->getNumMappers());

  for (long i_id = start_end_id.first; i_id < start_end_id.second; ++i_id) {
    std::shared_ptr<nvblox::Mapper> nvblox_mapper =
        mapper->getNvbloxMapper(i_id);

    const double voxel_size = nvblox_mapper->voxel_size_m();

    // TSDF Layer
    nvblox::TsdfLayer gt_tsdf(voxel_size, nvblox::MemoryType::kHost);
    // TODO(alexmillane, nvblox_torch_refactor): Take the TSDF distances as a
    // parameter.
    const float max_distance = 4.F * voxel_size;
    scene_->generateLayerFromScene<nvblox::TsdfVoxel>(max_distance, &gt_tsdf);

    // Copy to a GPU layer inside the mapper.
    nvblox_mapper->tsdf_layer().copyFrom(gt_tsdf);

    // Occupancy layer
    nvblox::OccupancyLayer gt_occupancy(voxel_size, nvblox::MemoryType::kHost);
    scene_->generateLayerFromScene<nvblox::OccupancyVoxel>(max_distance,
                                                           &gt_occupancy);
    nvblox_mapper->occupancy_layer().copyFrom(gt_occupancy);

    // We have updated all TSDF blocks in the mapper without the mapper's
    // knowledge. Update them all.
    nvblox_mapper->markBlocksForUpdate(
        nvblox_mapper->tsdf_layer().getAllBlockIndices());
    nvblox_mapper->markBlocksForUpdate(
        nvblox_mapper->occupancy_layer().getAllBlockIndices());

    // Generate the ESDF from everything in the TSDF.
    nvblox_mapper->updateEsdf();
  }
}

void Scene::createDummyMap() {
  // Create a map that's a box with a sphere in the middle.
  scene_->aabb() =
      nvblox::AxisAlignedBoundingBox(nvblox::Vector3f(-5.5f, -5.5f, -0.5f),
                                     nvblox::Vector3f(5.5f, 5.5f, 5.5f));
  scene_->addPlaneBoundaries(-5.0f, 5.0f, -5.0f, 5.0f);
  scene_->addGroundLevel(0.0f);
  scene_->addCeiling(5.0f);
  scene_->addPrimitive(std::make_unique<nvblox::primitives::Cube>(
      nvblox::Vector3f(0.0f, 0.0f, 2.0f), nvblox::Vector3f(2.0f, 2.0f, 2.0f)));
  scene_->addPrimitive(std::make_unique<nvblox::primitives::Sphere>(
      nvblox::Vector3f(0.0f, 0.0f, 2.0f), 2.0f));
}

}  // namespace pynvblox
