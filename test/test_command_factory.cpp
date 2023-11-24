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

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_following_utils/command_factory.hpp"

#include "../test/test_helper.h"

class TestCommandFactory : public ::testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}

  static void TearDownTestCase() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file", std::string(TEST_DIR) + "/test_command_factory.yaml"});
    node = std::make_shared<rclcpp::Node>("test_command_factory", options);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestCommandFactory, getParameterFollowMe)
{
  romea::core::FollowMe::Parameters params;

  romea::ros2::declare_follow_me_parameters(node, "follow_me");
  romea::ros2::get_params(node, "follow_me", params);

  EXPECT_DOUBLE_EQ(params.kp, 1.);
  EXPECT_DOUBLE_EQ(params.ki, 2.);
  EXPECT_DOUBLE_EQ(params.kd, 3.);
  EXPECT_DOUBLE_EQ(params.kdd, 4.);
}

TEST_F(TestCommandFactory, getParameterClassicSliding)
{
  romea::core::FollowTrajectoryClassicSliding::Parameters params;

  romea::ros2::declare_follow_trajectory_classic_sliding_parameters(node, "classic_sliding");
  romea::ros2::get_params(node, "classic_sliding", params);

  EXPECT_DOUBLE_EQ(params.front_kp, 5.);
  EXPECT_DOUBLE_EQ(params.rear_kp, 6.);
}

TEST_F(TestCommandFactory, getParameterPredictiveSliding)
{
  romea::core::FollowTrajectoryPredictiveSliding::Parameters params;

  romea::ros2::declare_follow_trajectory_predictive_sliding_parameters(node, "predictive_sliding");
  romea::ros2::get_params(node, "predictive_sliding", params);

  EXPECT_DOUBLE_EQ(params.front_kp, 7.);
  EXPECT_DOUBLE_EQ(params.rear_kp, 8.);
  EXPECT_DOUBLE_EQ(params.horizon, 9.);
  EXPECT_DOUBLE_EQ(params.a0, 10.);
  EXPECT_DOUBLE_EQ(params.a1, 11.);
  EXPECT_DOUBLE_EQ(params.b1, 12.);
  EXPECT_DOUBLE_EQ(params.b2, 13.);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
