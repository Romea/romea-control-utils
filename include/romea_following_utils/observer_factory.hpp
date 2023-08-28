// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef _ROMEA_FOLLOWING_UTILS__OBSERVER_FACTORY_HPP_
#define _ROMEA_FOLLOWING_UTILS__OBSERVER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_control/observer/SlidingObserverCinematicLinearTangent.hpp>
#include <romea_core_control/observer/SlidingObserverCinematicLyapounov.hpp>

namespace romea
{

template<typename Node>
void declare_sliding_observer_cinematic_linear_tangent_parameters(
  std::shared_ptr<Node> node, const std::string & params_ns)
{
  declare_parameter<double>(node, params_ns, "gains.lateral_deviation");
  declare_parameter<double>(node, params_ns, "gains.course_deviation");
  declare_parameter<double>(node, params_ns, "filter_weights.lateral_deviation");
  declare_parameter<double>(node, params_ns, "filter_weights.course_deviation");
  declare_parameter<double>(node, params_ns, "filter_weights.front_sliding_angle");
  declare_parameter<double>(node, params_ns, "filter_weights.rear_sliding_angle");
}

template<typename Node>
void declare_sliding_observer_cinematic_lyapounov_parameters(
  std::shared_ptr<Node> node, const std::string & params_ns)
{
  declare_parameter<double>(node, params_ns, "gains.x_deviation");
  declare_parameter<double>(node, params_ns, "gains.y_deviation");
  declare_parameter<double>(node, params_ns, "gains.course_deviation");
  declare_parameter<double>(node, params_ns, "gains.front_sliding_angle");
  declare_parameter<double>(node, params_ns, "gains.rear_sliding_angle");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, const std::string & params_ns,
  SlidingObserverCinematicLinearTangent::Parameters & parameters)
{
  parameters.lateralDeviationGain =
    get_parameter<double>(node, params_ns, "gains.lateral_deviation");
  parameters.courseDeviationGain = get_parameter<double>(node, params_ns, "gains.course_deviation");
  parameters.lateralDeviationFilterWeight =
    get_parameter<double>(node, params_ns, "filter_weights.lateral_deviation");
  parameters.courseDeviationFilterWeight =
    get_parameter<double>(node, params_ns, "filter_weights.course_deviation");
  parameters.frontSlidingAngleFilterWeight =
    get_parameter<double>(node, params_ns, "filter_weights.front_sliding_angle");
  parameters.rearSlidingAngleFilterWeight =
    get_parameter<double>(node, params_ns, "filter_weights.rear_sliding_angle");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, const std::string & params_ns,
  SlidingObserverCinematicLyapounov::Parameters & parameters)
{
  parameters.xDeviationGain = get_parameter<double>(node, params_ns, "gains.x_deviation");
  parameters.yDeviationGain = get_parameter<double>(node, params_ns, "gains.y_deviation");
  parameters.courseDeviationGain = get_parameter<double>(node, params_ns, "gains.course_deviation");
  parameters.frontSlidingAngleGain =
    get_parameter<double>(node, params_ns, "gains.front_sliding_angle");
  parameters.rearSlidingAngleGain =
    get_parameter<double>(node, params_ns, "gains.rear_sliding_angle");
}

template<typename ObserverType, typename Node, typename... Args>
std::unique_ptr<ObserverType> make_observer(
  std::shared_ptr<Node> node, const std::string & params_ns, Args &&... args)
{
  typename ObserverType::Parameters parameters;
  get_params(node, params_ns, parameters);
  return std::make_unique<ObserverType>(std::forward<Args>(args)..., parameters);
}

}  // namespace romea

#endif  // ROMEA_FOLLOWING_UTILS__OBSERVER_FACTORY_HPP_
