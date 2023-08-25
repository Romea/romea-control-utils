#include "ros/CommandFactory.hpp"
#include <ros/params/RosParam.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
void loadParams(ros::NodeHandle &nh, FollowMe::Parameters &parameters)
{
  parameters.kp=romea::loadParam<double>(nh,"gains/kp");
  parameters.ki=romea::loadParam<double>(nh,"gains/ki");
  parameters.kd=romea::loadParam<double>(nh,"gains/kd");
  parameters.kdd=romea::loadParam<double>(nh,"gains/kdd");
}

//-----------------------------------------------------------------------------
void loadParams(ros::NodeHandle &nh, FollowTrajectoryClassicSliding::Parameters & parameters)
{
  parameters.front_kp=loadParam<double>(nh,"gains/front_kp");
  parameters.rear_kp=loadParam<double>(nh,"gains/rear_kp");
}

//-----------------------------------------------------------------------------
void loadParams(ros::NodeHandle &nh, FollowTrajectoryPredictiveSliding::Parameters &parameters)
{
  parameters.front_kp=loadParam<double>(nh,"gains/front_kp");
  parameters.rear_kp=loadParam<double>(nh,"gains/rear_kp");
  parameters.horizon=loadParam<int>(nh,"prediction/horizon");
  parameters.a0=loadParam<double>(nh,"prediction/a0");
  parameters.a1=loadParam<double>(nh,"prediction/a1");
  parameters.b1=loadParam<double>(nh,"prediction/b1");
  parameters.b2=loadParam<double>(nh,"prediction/b2");
}

}
