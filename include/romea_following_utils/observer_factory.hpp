#ifndef _ROMEA_FOLLOWING_UTILS__OBSERVER_FACTORY_HPP_
#define _ROMEA_FOLLOWING_UTILS__OBSERVER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_control/observer/SlidingObserverCinematicLinearTangent.hpp>
#include <romea_core_control/observer/SlidingObserverCinematicLyapounov.hpp>

namespace romea
{

template<typename Node>
void declare_sliding_observer_cinematic_linear_tangent_parameters(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "gains.lateral_deviation");
  declare_parameter<double>(node, "gains.course_deviation");
  declare_parameter<double>(node, "filter_weights.lateral_deviation");
  declare_parameter<double>(node, "filter_weights.course_deviation");
  declare_parameter<double>(node, "filter_weights.front_sliding_angle");
  declare_parameter<double>(node, "filter_weights.rear_sliding_angle");
}

template<typename Node>
void declare_sliding_observer_cinematic_lyapounov_parameters(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "gains.x_deviation");
  declare_parameter<double>(node, "gains.y_deviation");
  declare_parameter<double>(node, "gains.course_deviation");
  declare_parameter<double>(node, "gains.front_sliding_angle");
  declare_parameter<double>(node, "gains.rear_sliding_angle");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, SlidingObserverCinematicLinearTangent::Parameters & parameters)
{
  parameters.lateralDeviationGain = get_parameter<double>(node, "gains.lateral_deviation");
  parameters.courseDeviationGain = get_parameter<double>(node, "gains.course_deviation");
  parameters.lateralDeviationFilterWeight =
    get_parameter<double>(node, "filter_weights.lateral_deviation");
  parameters.courseDeviationFilterWeight =
    get_parameter<double>(node, "filter_weights.course_deviation");
  parameters.frontSlidingAngleFilterWeight =
    get_parameter<double>(node, "filter_weights.front_sliding_angle");
  parameters.rearSlidingAngleFilterWeight =
    get_parameter<double>(node, "filter_weights.rear_sliding_angle");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, SlidingObserverCinematicLyapounov::Parameters & parameters)
{
  parameters.xDeviationGain = get_parameter<double>(node, "gains.x_deviation");
  parameters.yDeviationGain = get_parameter<double>(node, "gains.y_deviation");
  parameters.courseDeviationGain = get_parameter<double>(node, "gains.course_deviation");
  parameters.frontSlidingAngleGain = get_parameter<double>(node, "gains.front_sliding_angle");
  parameters.rearSlidingAngleGain = get_parameter<double>(node, "gains.rear_sliding_angle");
}

template<typename ObserverType, typename Node, typename... Args>
std::unique_ptr<ObserverType> make_observer(std::shared_ptr<Node> node, Args &&... args)
{
  typename ObserverType::Parameters parameters;
  get_params(node, parameters);
  return std::make_unique<ObserverType>(std::forward<Args>(args)..., parameters);
}

}  // namespace romea

#endif
