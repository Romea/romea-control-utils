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

#include <rclcpp/rclcpp.hpp>

// romea
#include <romea_following_utils/observer_factory.hpp>

#include "test_helper.h"

class TestObserverFactory : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file", std::string(TEST_DIR) + "/test_observer_factory.yaml"});
    node = std::make_shared<rclcpp::Node>("test_observer_factory", options);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestObserverFactory, getCinematicLinearTangentParams)
{
  romea::declare_sliding_observer_cinematic_linear_tangent_parameters(node, "linear_tangent");
  romea::SlidingObserverCinematicLinearTangent::Parameters params;
  romea::get_params(node, "linear_tangent", params);

  EXPECT_DOUBLE_EQ(params.lateralDeviationGain, 1.);
  EXPECT_DOUBLE_EQ(params.courseDeviationGain, 2.);
  EXPECT_DOUBLE_EQ(params.lateralDeviationFilterWeight, 3.);
  EXPECT_DOUBLE_EQ(params.courseDeviationFilterWeight, 4.);
  EXPECT_DOUBLE_EQ(params.frontSlidingAngleFilterWeight, 5.);
  EXPECT_DOUBLE_EQ(params.rearSlidingAngleFilterWeight, 6.);
}

TEST_F(TestObserverFactory, getCinematicLyapounovParams)
{
  romea::declare_sliding_observer_cinematic_lyapunov_parameters(node, "lyapunov");
  romea::SlidingObserverCinematicLyapounov::Parameters params;
  romea::get_params(node, "lyapunov", params);

  EXPECT_DOUBLE_EQ(params.xDeviationGain, 7.);
  EXPECT_DOUBLE_EQ(params.yDeviationGain, 8.);
  EXPECT_DOUBLE_EQ(params.courseDeviationGain, 9.);
  EXPECT_DOUBLE_EQ(params.frontSlidingAngleGain, 10.);
  EXPECT_DOUBLE_EQ(params.rearSlidingAngleGain, 11.);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
