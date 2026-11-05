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

#ifndef KORTEX_EFFORT_CONTROLLER__KORTEX_EFFORT_CONTROLLER_HPP_
#define KORTEX_EFFORT_CONTROLLER__KORTEX_EFFORT_CONTROLLER_HPP_

#include <limits>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace kortex_effort_controller
{

/// The latest joint-torque command and when it arrived.
struct TimedCommand
{
  std::vector<double> torques;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

enum class CommandAge
{
  NONE,   ///< nothing received yet
  FRESH,  ///< younger than the warn threshold
  STALE_WARN,  ///< older than the warn threshold, still used
  TIMED_OUT,   ///< older than the timeout: output NaN
};

/// Watchdog decision, kept free of ROS objects for testing.
CommandAge classify_command(
  const TimedCommand & cmd, const rclcpp::Time & now, double warn_sec, double timeout_sec);

/// Forwards Float64MultiArray joint torques (~/commands) to the joints' effort
/// command interfaces, like effort_controllers/JointGroupEffortController,
/// but with a watchdog: before the first command, and whenever the newest
/// one is older than cmd_timeout_sec, every joint gets NaN. The Kortex
/// hardware interface treats NaN as "stop": actuators back to POSITION mode
/// (a position hold), latched until this controller is restarted. A plain
/// forward controller would keep applying the last torque forever if its
/// publisher died.
class KortexEffortController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void write_all(double value);

  std::vector<std::string> joints_;
  double cmd_timeout_sec_ = 0.2;
  double cmd_stale_warn_sec_ = 0.05;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
  realtime_tools::RealtimeThreadSafeBox<TimedCommand> latest_;
  TimedCommand rt_command_;  // update()'s own copy
  bool warned_stale_ = false;
  bool reported_timeout_ = false;
};

}  // namespace kortex_effort_controller

#endif  // KORTEX_EFFORT_CONTROLLER__KORTEX_EFFORT_CONTROLLER_HPP_
