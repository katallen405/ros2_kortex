// Copyright 2024, Your Name
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

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace kortex_effort_controller
{

class KortexEffortController : public controller_interface::ControllerInterface
{
public:
  KortexEffortController();

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

private:
  // Joint names claimed from the hardware interface
  std::vector<std::string> joint_names_;

  // Per-joint torque limits (Nm) — checked against hardware limits at configure time
  std::vector<double> torque_limits_;

  // Latest commanded torques, protected by a mutex
  std::mutex torque_mutex_;
  std::vector<double> desired_torques_;
  bool has_command_{false};

  // ROS2 subscription
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;

  void torque_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace kortex_effort_controller

#endif  // KORTEX_EFFORT_CONTROLLER__KORTEX_EFFORT_CONTROLLER_HPP_
