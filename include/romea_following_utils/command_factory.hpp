#ifndef _ROMEA_FOLLOWING_UTILS__COMMAND_FACTORY_HPP_
#define _ROMEA_FOLLOWING_UTILS__COMMAND_FACTORY_HPP_

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include <romea_core_control/command/FollowMe.hpp>
#include <romea_core_control/command/FollowTrajectoryClassicSliding.hpp>
#include <romea_core_control/command/FollowTrajectoryPredictiveSliding.hpp>
#include <romea_common_utils/params/node_parameters.hpp>

namespace romea
{

template<typename Node>
void declare_follow_me_parameters(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "gains.kp");
  declare_parameter<double>(node, "gains.ki");
  declare_parameter<double>(node, "gains.kd");
  declare_parameter<double>(node, "gains.kdd");
}

template<typename Node>
void declare_follow_trajectory_classic_sliding_parameters(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "gains.front_kp");
  declare_parameter<double>(node, "gains.rear_kp");
}

template<typename Node>
void declare_follow_trajectory_predictive_sliding_parameters(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "gains.front_kp");
  declare_parameter<double>(node, "gains.rear_kp");
  declare_parameter<double>(node, "prediction.horizon");
  declare_parameter<double>(node, "prediction.a0");
  declare_parameter<double>(node, "prediction.a1");
  declare_parameter<double>(node, "prediction.b1");
  declare_parameter<double>(node, "prediction.b2");
}

template<typename Node>
void get_params(std::shared_ptr<Node> node, FollowMe::Parameters & parameters)
{
  parameters.kp = get_parameter<double>(node, "gains.kp");
  parameters.ki = get_parameter<double>(node, "gains.ki");
  parameters.kd = get_parameter<double>(node, "gains.kd");
  parameters.kdd = get_parameter<double>(node, "gains.kdd");
}

template<typename Node>
void get_params(std::shared_ptr<Node> node, FollowTrajectoryClassicSliding::Parameters & parameters)
{
  parameters.front_kp = get_parameter<double>(node, "gains.front_kp");
  parameters.rear_kp = get_parameter<double>(node, "gains.rear_kp");
}

template<typename Node>
void get_params(
  std::shared_ptr<Node> node, FollowTrajectoryPredictiveSliding::Parameters & parameters)
{
  parameters.front_kp = get_parameter<double>(node, "gains.front_kp");
  parameters.rear_kp = get_parameter<double>(node, "gains.rear_kp");
  parameters.horizon = get_parameter<double>(node, "prediction.horizon");
  parameters.a0 = get_parameter<double>(node, "prediction.a0");
  parameters.a1 = get_parameter<double>(node, "prediction.a1");
  parameters.b1 = get_parameter<double>(node, "prediction.b1");
  parameters.b2 = get_parameter<double>(node, "prediction.b2");
}

template<typename CommandType, typename Node, typename... Args>
std::unique_ptr<CommandType> make_command(std::shared_ptr<Node> node, Args &&... args)
{
  typename CommandType::Parameters parameters;
  get_params(node, parameters);
  return std::make_unique<CommandType>(std::forward<Args>(args)..., parameters);
}

}  // namespace romea

#endif
