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

#ifndef _ROMEA_FOLLOWING_UTILS__INERTIA_FACTORY_HPP_
#define _ROMEA_FOLLOWING_UTILS__INERTIA_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <romea_common_utils/params/eigen_parameters.hpp>
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_control/InertiaParameters.hpp>

namespace romea
{

template<typename Node>
void declare_inertia_params(std::shared_ptr<Node> node, const std::string & params_ns)
{
  declare_parameter<double>(node, params_ns, "chassis.mass");
  declare_eigen_vector_parameter<Eigen::Vector3d>(node, params_ns, "chassis.mass_center_position");
  declare_parameter<double>(node, params_ns, "kinematic.z_inertial_moment");
}

template<typename Node>
void get_inertia_params(
  std::shared_ptr<Node> node, const std::string & params_ns, InertiaParameters & parameters)
{
  parameters.mass = get_parameter<double>(node, params_ns, "chassis.mass");
  parameters.massCenter =
    get_eigen_vector_parameter<Eigen::Vector3d>(node, params_ns, "chassis.mass_center_position");
  parameters.zInertialMoment =
    get_parameter<double>(node, params_ns, "kinematic.z_inertial_moment");
}

template<typename Node>
InertiaParameters get_inertia_params(std::shared_ptr<Node> node, const std::string & params_ns)
{
  InertiaParameters parameters;
  get_inertia_params(std::move(node), params_ns, parameters);
  return parameters;
}

}  // namespace romea

#endif  // ROMEA_FOLLOWING_UTILS__INERTIA_FACTORY_HPP_
