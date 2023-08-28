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

#ifndef _ROMEA_FOLLOWING_UTILS__COMMAND_FACTORY_HPP_
#define _ROMEA_FOLLOWING_UTILS__COMMAND_FACTORY_HPP_

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_control/command/FollowMe.hpp>
#include <romea_core_control/command/FollowTrajectoryClassicSliding.hpp>
#include <romea_core_control/command/FollowTrajectoryPredictiveSliding.hpp>

namespace romea
{

template<typename Node>
void declare_follow_me_parameters(std::shared_ptr<Node> node, const std::string & params_ns)
{
  declare_parameter<double>(node, params_ns, "gains.kp");
  declare_parameter<double>(node, params_ns, "gains.ki");
  declare_parameter<double>(node, params_ns, "gains.kd");
  declare_parameter<double>(node, params_ns, "gains.kdd");
}

template<typename Node>
void declare_follow_trajectory_classic_sliding_parameters(
  std::shared_ptr<Node> node, const std::string & params_ns)
{
  declare_parameter<double>(node, params_ns, "gains.front_kp");
  declare_parameter<double>(node, params_ns, "gains.rear_kp");
}

template<typename Node>
void declare_follow_trajectory_predictive_sliding_parameters(
  std::shared_ptr<Node> node, const std::string & params_ns)
{
  declare_parameter<double>(node, params_ns, "gains.front_kp");
  declare_parameter<double>(node, params_ns, "gains.rear_kp");
  declare_parameter<double>(node, params_ns, "prediction.horizon");
  declare_parameter<double>(node, params_ns, "prediction.a0");
  declare_parameter<double>(node, params_ns, "prediction.a1");
  declare_parameter<double>(node, params_ns, "prediction.b1");
  declare_parameter<double>(node, params_ns, "prediction.b2");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, const std::string & params_ns, FollowMe::Parameters & parameters)
{
  parameters.kp = get_parameter<double>(node, params_ns, "gains.kp");
  parameters.ki = get_parameter<double>(node, params_ns, "gains.ki");
  parameters.kd = get_parameter<double>(node, params_ns, "gains.kd");
  parameters.kdd = get_parameter<double>(node, params_ns, "gains.kdd");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, const std::string & params_ns,
  FollowTrajectoryClassicSliding::Parameters & parameters)
{
  parameters.front_kp = get_parameter<double>(node, params_ns, "gains.front_kp");
  parameters.rear_kp = get_parameter<double>(node, params_ns, "gains.rear_kp");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, const std::string & params_ns,
  FollowTrajectoryPredictiveSliding::Parameters & parameters)
{
  parameters.front_kp = get_parameter<double>(node, params_ns, "gains.front_kp");
  parameters.rear_kp = get_parameter<double>(node, params_ns, "gains.rear_kp");
  parameters.horizon = get_parameter<double>(node, params_ns, "prediction.horizon");
  parameters.a0 = get_parameter<double>(node, params_ns, "prediction.a0");
  parameters.a1 = get_parameter<double>(node, params_ns, "prediction.a1");
  parameters.b1 = get_parameter<double>(node, params_ns, "prediction.b1");
  parameters.b2 = get_parameter<double>(node, params_ns, "prediction.b2");
}

template<typename CommandType, typename Node, typename... Args>
std::unique_ptr<CommandType> make_command(
  std::shared_ptr<Node> node, const std::string & params_ns, Args &&... args)
{
  typename CommandType::Parameters parameters;
  get_params(node, params_ns, parameters);
  return std::make_unique<CommandType>(std::forward<Args>(args)..., parameters);
}

}  // namespace romea

#endif  // ROMEA_FOLLOWING_UTILS__COMMAND_FACTORY_HPP_
