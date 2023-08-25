#ifndef _ROMEA_FOLLOWING_UTILS__INERTIA_FACTORY_HPP_
#define _ROMEA_FOLLOWING_UTILS__INERTIA_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <romea_core_control/InertiaParameters.hpp>
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_common_utils/params/eigen_parameters.hpp>

namespace romea
{

template<typename Node>
void declare_inertia_params(std::shared_ptr<Node> node, const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "chassis.mass");
  declare_eigen_vector_parameter(node, parameters_ns, "chassis.mass_center_position");
  declare_parameter<double>(node, parameters_ns, "kinematic.z_inertial_moment");
}

template<typename Node>
void get_inertia_params(std::shared_ptr<Node> node, InertiaParameters & parameters)
{
  parameters.mass = get_parameter<double>(node, "chassis.mass");
  parameters.massCenter = get_eigen_vector_parameter(node, "chassis.mass_center_position");
  parameters.zInertialMoment = get_parameter<double>(node, "kinematic.z_inertial_moment");
}

template<typename Node>
InertiaParameters get_inertia_params(std::shared_ptr<Node> node)
{
  InertiaParameters parameters;
  get_inertia_params(std::move(node), parameters);
  return parameters;
}

}  // namespace romea

#endif
