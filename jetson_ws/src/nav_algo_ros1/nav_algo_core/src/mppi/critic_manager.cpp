// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav_algo_core/compat.hpp"
#include "nav_algo_core/mppi/critic_manager.hpp"

// Direct includes of the 8 ported critics (replacing Nav2's pluginlib-loaded
// set). Order matters only at compile time, not at runtime.
#include "nav_algo_core/mppi/critics/constraint_critic.hpp"
#include "nav_algo_core/mppi/critics/goal_angle_critic.hpp"
#include "nav_algo_core/mppi/critics/goal_critic.hpp"
#include "nav_algo_core/mppi/critics/obstacles_critic.hpp"
#include "nav_algo_core/mppi/critics/path_align_critic.hpp"
#include "nav_algo_core/mppi/critics/path_angle_critic.hpp"
#include "nav_algo_core/mppi/critics/path_follow_critic.hpp"
#include "nav_algo_core/mppi/critics/prefer_forward_critic.hpp"

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  // Direct instantiation, replacing Nav2's pluginlib-based loadCritics().
  // Under ROS 1 the critics live in a static lib (libnav_algo_mppi.a) with
  // no pluginlib XML. Compile-time if-chain over the 8 critic types in the
  // ported set keeps behaviour identical: same on_configure() signature,
  // same parameter namespace, same cost evaluation order.
  critics_.clear();
  for (auto name : critic_names_) {
    std::unique_ptr<critics::CriticFunction> instance;
    if      (name == "ConstraintCritic")    instance = std::make_unique<critics::ConstraintCritic>();
    else if (name == "GoalCritic")          instance = std::make_unique<critics::GoalCritic>();
    else if (name == "GoalAngleCritic")     instance = std::make_unique<critics::GoalAngleCritic>();
    else if (name == "ObstaclesCritic")     instance = std::make_unique<critics::ObstaclesCritic>();
    else if (name == "PathAlignCritic")     instance = std::make_unique<critics::PathAlignCritic>();
    else if (name == "PathFollowCritic")    instance = std::make_unique<critics::PathFollowCritic>();
    else if (name == "PathAngleCritic")     instance = std::make_unique<critics::PathAngleCritic>();
    else if (name == "PreferForwardCritic") instance = std::make_unique<critics::PreferForwardCritic>();
    else {
      RCLCPP_WARN(logger_,
        "Critic '%s' not in the static-linked set; skipping.", name.c_str());
      continue;
    }
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", name.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }
    critics_[q]->score(data);
  }
}

}  // namespace mppi
