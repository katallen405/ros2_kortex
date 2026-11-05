// Copyright 2026, Tufts University AABL Lab.
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

#include <gmock/gmock.h>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kortex_effort_controller/kortex_effort_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

using kortex_effort_controller::classify_command;
using kortex_effort_controller::CommandAge;
using kortex_effort_controller::KortexEffortController;
using kortex_effort_controller::TimedCommand;

namespace
{
rclcpp::Time t(double sec) { return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME); }

TimedCommand cmd_at(double sec)
{
  TimedCommand c;
  c.torques = {1.0, 2.0};
  c.stamp = t(sec);
  c.valid = true;
  return c;
}
}  // namespace

TEST(ClassifyCommand, Ages)
{
  EXPECT_EQ(classify_command(TimedCommand(), t(10.0), 0.05, 0.2), CommandAge::NONE);
  EXPECT_EQ(classify_command(cmd_at(10.0), t(10.01), 0.05, 0.2), CommandAge::FRESH);
  EXPECT_EQ(classify_command(cmd_at(10.0), t(10.1), 0.05, 0.2), CommandAge::STALE_WARN);
  EXPECT_EQ(classify_command(cmd_at(10.0), t(10.3), 0.05, 0.2), CommandAge::TIMED_OUT);
}

/// Exposes the protected subscription callback so tests can inject commands
/// without a ROS graph.
class TestableController : public KortexEffortController
{
public:
  void inject(const std::vector<double> & torques)
  {
    auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    msg->data = torques;
    command_callback(msg);
  }
};

class ControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "effort_controller";
    params.update_rate = 1000;
    params.controller_manager_update_rate = 1000;
    params.node_options = controller_->define_custom_node_options();
    params.node_options.parameter_overrides(
      {{"joints", std::vector<std::string>{"joint_1", "joint_2"}},
       {"cmd_timeout_sec", 0.2},
       {"cmd_stale_warn_sec", 0.05}});
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    std::vector<hardware_interface::LoanedCommandInterface> loaned;
    loaned.emplace_back(joint1_);
    loaned.emplace_back(joint2_);
    controller_->assign_interfaces(std::move(loaned), {});
    ASSERT_EQ(
      controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  double value(const hardware_interface::CommandInterface::SharedPtr & ci)
  {
    return ci->get_optional().value();
  }

  controller_interface::return_type update_at(double sec)
  {
    return controller_->update(t(sec), rclcpp::Duration::from_seconds(0.001));
  }

  std::unique_ptr<TestableController> controller_;
  hardware_interface::CommandInterface::SharedPtr joint1_ =
    std::make_shared<hardware_interface::CommandInterface>(
      "joint_1", hardware_interface::HW_IF_EFFORT, &joint1_value_);
  hardware_interface::CommandInterface::SharedPtr joint2_ =
    std::make_shared<hardware_interface::CommandInterface>(
      "joint_2", hardware_interface::HW_IF_EFFORT, &joint2_value_);
  double joint1_value_ = 0.0;
  double joint2_value_ = 0.0;
};

TEST_F(ControllerTest, NanBeforeFirstCommand)
{
  ASSERT_EQ(update_at(1.0), controller_interface::return_type::OK);
  EXPECT_TRUE(std::isnan(value(joint1_)));
  EXPECT_TRUE(std::isnan(value(joint2_)));
}

TEST_F(ControllerTest, FreshCommandPassesThrough)
{
  controller_->inject({1.5, -2.5});
  const double now = controller_->get_node()->now().seconds();
  update_at(now + 0.01);
  EXPECT_DOUBLE_EQ(value(joint1_), 1.5);
  EXPECT_DOUBLE_EQ(value(joint2_), -2.5);
}

TEST_F(ControllerTest, StaleCommandBecomesNanAndStaysStopped)
{
  controller_->inject({1.5, -2.5});
  const double now = controller_->get_node()->now().seconds();
  update_at(now + 0.5);  // past the 0.2 s timeout
  EXPECT_TRUE(std::isnan(value(joint1_)));
  controller_->inject({3.0, 3.0});  // the stream comes back...
  update_at(controller_->get_node()->now().seconds() + 0.01);
  EXPECT_TRUE(std::isnan(value(joint1_)));  // ...but stays stopped until restarted
}

TEST_F(ControllerTest, WrongLengthAndNanCommandsIgnored)
{
  controller_->inject({1.0, 2.0, 3.0});
  controller_->inject({NAN, 2.0});
  update_at(controller_->get_node()->now().seconds() + 0.01);
  EXPECT_TRUE(std::isnan(value(joint1_)));  // still nothing valid received
}

TEST_F(ControllerTest, ReactivationForgetsOldCommands)
{
  controller_->inject({1.5, -2.5});
  ASSERT_EQ(
    controller_->get_node()->deactivate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(std::isnan(value(joint1_)));
  ASSERT_EQ(
    controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  update_at(controller_->get_node()->now().seconds() + 0.01);
  EXPECT_TRUE(std::isnan(value(joint1_)));
}
