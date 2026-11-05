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

#include "kortex_effort_controller/kortex_effort_controller.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace kortex_effort_controller
{

CommandAge classify_command(
  const TimedCommand & cmd, const rclcpp::Time & now, double warn_sec, double timeout_sec)
{
  if (!cmd.valid)
  {
    return CommandAge::NONE;
  }
  const double age = (now - cmd.stamp).seconds();
  if (age > timeout_sec)
  {
    return CommandAge::TIMED_OUT;
  }
  return age > warn_sec ? CommandAge::STALE_WARN : CommandAge::FRESH;
}

controller_interface::CallbackReturn KortexEffortController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<double>("cmd_timeout_sec", 0.2);
    auto_declare<double>("cmd_stale_warn_sec", 0.05);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KortexEffortController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  cmd_timeout_sec_ = get_node()->get_parameter("cmd_timeout_sec").as_double();
  cmd_stale_warn_sec_ = get_node()->get_parameter("cmd_stale_warn_sec").as_double();
  if (joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (cmd_timeout_sec_ <= 0.0 || cmd_stale_warn_sec_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "cmd_timeout_sec and cmd_stale_warn_sec must be > 0.");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Created here, not in on_activate: commands that arrive while the
  // controller is still inactive count, so a publisher that is already
  // streaming makes the first active update() fresh.
  subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { command_callback(msg); });
  RCLCPP_INFO(
    get_node()->get_logger(), "Configured for %zu joints; watchdog %.3fs (warn %.3fs).",
    joints_.size(), cmd_timeout_sec_, cmd_stale_warn_sec_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
KortexEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
KortexEffortController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

void KortexEffortController::command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joints_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Command has %zu values, expected %zu. Ignoring.", msg->data.size(), joints_.size());
    return;
  }
  for (const double value : msg->data)
  {
    if (!std::isfinite(value))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Command contains NaN/inf. Ignoring.");
      return;
    }
  }
  TimedCommand cmd;
  cmd.torques = msg->data;
  cmd.stamp = get_node()->now();
  cmd.valid = true;
  latest_.set(cmd);
}

controller_interface::CallbackReturn KortexEffortController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Only commands newer than activation count: a stream that stopped while
  // this controller was inactive must not be replayed.
  latest_.set([](TimedCommand & cmd) { cmd.valid = false; });
  rt_command_ = TimedCommand();
  warned_stale_ = false;
  reported_timeout_ = false;
  write_all(std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KortexEffortController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  write_all(std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

void KortexEffortController::write_all(double value)
{
  for (auto & interface : command_interfaces_)
  {
    (void)interface.set_value(value);
  }
}

controller_interface::return_type KortexEffortController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Non-blocking: if the subscriber holds the lock right now, use the copy
  // from the previous cycle (at most one cycle older).
  if (auto latest = latest_.try_get())
  {
    rt_command_ = *latest;
  }

  switch (classify_command(rt_command_, time, cmd_stale_warn_sec_, cmd_timeout_sec_))
  {
    case CommandAge::NONE:
      write_all(std::numeric_limits<double>::quiet_NaN());
      return controller_interface::return_type::OK;
    case CommandAge::TIMED_OUT:
      if (!reported_timeout_)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "No torque command for more than %.3fs -- commanding NaN (hardware stops effort "
          "control). Restart this controller to resume.",
          cmd_timeout_sec_);
        reported_timeout_ = true;
      }
      write_all(std::numeric_limits<double>::quiet_NaN());
      return controller_interface::return_type::OK;
    case CommandAge::STALE_WARN:
      if (!warned_stale_)
      {
        RCLCPP_WARN(
          get_node()->get_logger(), "Torque command older than %.0fms (timeout %.0fms).",
          cmd_stale_warn_sec_ * 1000.0, cmd_timeout_sec_ * 1000.0);
        warned_stale_ = true;
      }
      break;
    case CommandAge::FRESH:
      warned_stale_ = false;
      break;
  }

  // A timed-out stream stays stopped even if commands resume: the hardware
  // has already left TORQUE mode, and resuming silently would surprise.
  if (reported_timeout_)
  {
    write_all(std::numeric_limits<double>::quiet_NaN());
    return controller_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < command_interfaces_.size(); i++)
  {
    (void)command_interfaces_[i].set_value(rt_command_.torques[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace kortex_effort_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_effort_controller::KortexEffortController, controller_interface::ControllerInterface)
