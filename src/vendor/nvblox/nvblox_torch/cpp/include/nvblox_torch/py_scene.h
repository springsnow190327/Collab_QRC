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

#include <nvblox/primitives/scene.h>
#include <nvblox_torch/py_mapper.h>

namespace pynvblox {

struct Scene : torch::CustomClassHolder {
  // TODO: wrap the remaining functions in nvblox scene.h if we care about them
  // at all

  Scene() { scene_ = std::make_shared<nvblox::primitives::Scene>(); };

  std::shared_ptr<nvblox::primitives::Scene> scene_;

  void setAABB(std::vector<double> low, std::vector<double> high);

  std::tuple<std::vector<double>, std::vector<double>> getAABB();

  void addPlaneBoundaries(double x_min, double x_max, double y_min,
                          double y_max);

  void addGroundLevel(double level);

  void addCeiling(double ceiling);

  void addPrimitive(std::string type, std::vector<double> prim_params);

  std::vector<std::string> getPrimitiveTypesList() const;

  void createDummyMap();

  c10::intrusive_ptr<Scene> clone() const {
    return c10::make_intrusive<Scene>();
  }

  void toMapper(c10::intrusive_ptr<Mapper> mapper, long mapper_id = -1);
};
}  // namespace pynvblox
