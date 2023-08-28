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
#include <romea_following_utils/inertia_factory.hpp>

#include "test_helper.h"

class TestInertiaFactory : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file", std::string(TEST_DIR) + "/test_inertia_factory.yaml"});
    node = std::make_shared<rclcpp::Node>("test_inertia_factory", options);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestInertiaFactory, getParameters)
{
  romea::declare_inertia_params(node, "inertia");
  auto params = romea::get_inertia_params(node, "inertia");

  EXPECT_DOUBLE_EQ(params.mass, 1.);
  EXPECT_DOUBLE_EQ(params.massCenter[0], 2.);
  EXPECT_DOUBLE_EQ(params.massCenter[1], 3.);
  EXPECT_DOUBLE_EQ(params.massCenter[2], 4.);
  EXPECT_DOUBLE_EQ(params.zInertialMoment, 5.);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
