/**
 * kortex_torque_node.cpp
 *
 * ROS2 node that subscribes to /joint_torque_command (std_msgs/Float64MultiArray,
 * 7 elements in Nm) and drives a Kinova Gen3 7-DOF arm via the Kortex low-level
 * cyclic API (BaseCyclic::Refresh at up to 1 kHz).
 *
 * Topics
 * ------
 *   ~/joint_torque_command  [std_msgs/Float64MultiArray]  in  - desired torques (Nm)
 *   ~/joint_torque_feedback [std_msgs/Float64MultiArray]  out - measured torques (Nm)
 *
 * Parameters
 * ----------
 *   robot_ip          (string,  "192.168.1.10")
 *   username          (string,  "admin")
 *   password          (string,  "admin")
 *   dof               (int,     7)
 *   cyclic_period_ms  (int,     1)       1–25 ms
 *   torque_topic      (string,  "~/joint_torque_command")
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Kortex API
#include "ActuatorConfigClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "BaseClientRpc.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"
#include "ActuatorConfig.pb.h"
#include "Base.pb.h"
#include "BaseCyclic.pb.h"
#include "Session.pb.h"

namespace k_api = Kinova::Api;

// Conservative torque limits (Nm): joints 1-4 large, 5-7 small
static const std::vector<double> MAX_TORQUE_NM = {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};

class KortexTorqueBridgeNode : public rclcpp::Node
{
public:
  KortexTorqueBridgeNode()
  : Node("kortex_torque_bridge")
  {
    // ── Parameters ────────────────────────────────────────────────────────
    declare_parameter("robot_ip",         "192.168.1.10");
    declare_parameter("username",         "admin");
    declare_parameter("password",         "admin");
    declare_parameter("dof",              7);
    declare_parameter("cyclic_period_ms", 1);
    declare_parameter("torque_topic",     "~/joint_torque_command");

    robot_ip_      = get_parameter("robot_ip").as_string();
    username_      = get_parameter("username").as_string();
    password_      = get_parameter("password").as_string();
    dof_           = get_parameter("dof").as_int();
    period_ms_     = get_parameter("cyclic_period_ms").as_int();
    torque_topic_  = get_parameter("torque_topic").as_string();

    desired_torques_.assign(dof_, 0.0);

    // ── ROS2 interfaces ───────────────────────────────────────────────────
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      torque_topic_, 10,
      std::bind(&KortexTorqueBridgeNode::torque_callback, this, std::placeholders::_1));

    pub_feedback_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/joint_torque_feedback", 10);

    RCLCPP_INFO(get_logger(), "Subscribed to %s", torque_topic_.c_str());

    // ── Connect & initialise ──────────────────────────────────────────────
    connect();
    init_low_level();

    // ── Start cyclic thread ───────────────────────────────────────────────
    running_ = true;
    cyclic_thread_ = std::thread(&KortexTorqueBridgeNode::cyclic_loop, this);
    RCLCPP_INFO(get_logger(), "Cyclic torque control thread started at %d ms period.", period_ms_);
  }

  ~KortexTorqueBridgeNode()
  {
    shutdown();
  }

private:
  // ── Kortex connection ──────────────────────────────────────────────────

  void connect()
  {
    RCLCPP_INFO(get_logger(), "Connecting to robot at %s …", robot_ip_.c_str());

    tcp_transport_ = std::make_unique<k_api::TransportClientTcp>();
    tcp_transport_->connect(robot_ip_, 10000);
    tcp_router_ = std::make_unique<k_api::RouterClient>(
      tcp_transport_.get(), [](k_api::KError err) {
        fprintf(stderr, "Kortex TCP error: %s\n", err.toString().c_str());
      });

    udp_transport_ = std::make_unique<k_api::TransportClientUdp>();
    udp_transport_->connect(robot_ip_, 10001);
    udp_router_ = std::make_unique<k_api::RouterClient>(
      udp_transport_.get(), [](k_api::KError err) {
        fprintf(stderr, "Kortex UDP error: %s\n", err.toString().c_str());
      });

    // Session on TCP router
    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username(username_);
    session_info.set_password(password_);
    session_info.set_session_inactivity_timeout(60000);
    session_info.set_connection_inactivity_timeout(2000);

    session_manager_ = std::make_unique<k_api::SessionManager>(tcp_router_.get());
    session_manager_->CreateSession(session_info);

    base_          = std::make_unique<k_api::Base::BaseClient>(tcp_router_.get());
    base_cyclic_   = std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(udp_router_.get());
    actuator_cfg_  = std::make_unique<k_api::ActuatorConfig::ActuatorConfigClient>(tcp_router_.get());

    RCLCPP_INFO(get_logger(), "Connected.");
  }

  // ── Low-level init ────────────────────────────────────────────────────

  void init_low_level()
  {
    // Save current servoing mode for restoration on shutdown
    prev_servoing_mode_ = base_->GetServoingMode();

    // Switch to LOW_LEVEL_SERVOING
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(servoing_mode);

    // Seed command with current feedback
    auto feedback = base_cyclic_->RefreshFeedback();
    int actuator_count = base_->GetActuatorCount().count();

    for (int i = 0; i < actuator_count; ++i) {
      auto cmd = base_command_.add_actuators();
      cmd->set_position(feedback.actuators(i).position());
      cmd->set_velocity(0.0f);
      cmd->set_torque_joint(0.0f);
      cmd->set_command_id(0);
    }

    // Switch each actuator to TORQUE control mode (1-indexed)
    auto ctrl_mode = k_api::ActuatorConfig::ControlModeInformation();
    ctrl_mode.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    for (int joint_id = 1; joint_id <= dof_; ++joint_id) {
      actuator_cfg_->SetControlMode(ctrl_mode, joint_id);
    }

    RCLCPP_INFO(get_logger(), "All actuators set to TORQUE control mode.");
  }

  // ── ROS2 subscriber callback ──────────────────────────────────────────

  void torque_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (static_cast<int>(msg->data.size()) != dof_) {
      RCLCPP_WARN(get_logger(),
        "Expected %d torque values, got %zu. Ignoring.", dof_, msg->data.size());
      return;
    }

    std::vector<double> clamped(dof_);
    for (int i = 0; i < dof_; ++i) {
      double limit = (i < static_cast<int>(MAX_TORQUE_NM.size())) ? MAX_TORQUE_NM[i] : 9.0;
      double tau   = msg->data[i];
      double safe  = std::max(-limit, std::min(limit, tau));
      if (std::abs(safe) < std::abs(tau)) {
        RCLCPP_WARN(get_logger(),
          "Joint %d torque %.2f Nm clamped to %.2f Nm", i, tau, safe);
      }
      clamped[i] = safe;
    }

    std::lock_guard<std::mutex> lock(torque_mutex_);
    desired_torques_ = clamped;
  }

  // ── Cyclic loop ───────────────────────────────────────────────────────

  void cyclic_loop()
  {
    auto send_options = k_api::RouterClientSendOptions();
    send_options.timeout_ms = 3;  // tight deadline for cyclic

    int feedback_counter = 0;

    while (running_) {
      auto loop_start = std::chrono::steady_clock::now();

      try {
        auto feedback = base_cyclic_->Refresh(base_command_, 0, send_options);

        std::vector<double> desired;
        {
          std::lock_guard<std::mutex> lock(torque_mutex_);
          desired = desired_torques_;
        }

        std_msgs::msg::Float64MultiArray fb_msg;
        fb_msg.data.resize(dof_);

        for (int i = 0; i < dof_; ++i) {
          // Mirror measured position back to suppress following-error watchdog
          base_command_.mutable_actuators(i)->set_position(
            feedback.actuators(i).position());
          // Inject desired torque
          base_command_.mutable_actuators(i)->set_torque_joint(
            static_cast<float>(desired[i]));

          fb_msg.data[i] = feedback.actuators(i).torque();
        }

        // Publish feedback at ~50 Hz (every 20 cycles at 1 ms period)
        if (++feedback_counter >= 20) {
          feedback_counter = 0;
          pub_feedback_->publish(fb_msg);
        }

      } catch (const k_api::KDetailedException & ex) {
        RCLCPP_ERROR(get_logger(), "Kortex cyclic error: %s", ex.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Cyclic loop exception: %s", ex.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      auto elapsed = std::chrono::steady_clock::now() - loop_start;
      auto sleep_time = std::chrono::milliseconds(period_ms_) - elapsed;
      if (sleep_time > std::chrono::milliseconds(0)) {
        std::this_thread::sleep_for(sleep_time);
      }
    }
  }

  // ── Shutdown ──────────────────────────────────────────────────────────

  void shutdown()
  {
    RCLCPP_INFO(get_logger(), "Shutting down kortex_torque_bridge …");
    running_ = false;
    if (cyclic_thread_.joinable()) {
      cyclic_thread_.join();
    }

    try {
      // Zero all torques before switching back
      for (int i = 0; i < dof_; ++i) {
        base_command_.mutable_actuators(i)->set_torque_joint(0.0f);
      }
      base_cyclic_->Refresh(base_command_);

      // Restore POSITION control mode
      auto ctrl_mode = k_api::ActuatorConfig::ControlModeInformation();
      ctrl_mode.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
      for (int joint_id = 1; joint_id <= dof_; ++joint_id) {
        actuator_cfg_->SetControlMode(ctrl_mode, joint_id);
      }

      // Restore previous servoing mode
      base_->SetServoingMode(prev_servoing_mode_);
      RCLCPP_INFO(get_logger(), "Robot restored to previous servoing mode.");
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Cleanup error (non-fatal): %s", ex.what());
    }

    try {
      session_manager_->CloseSession();
      tcp_transport_->disconnect();
      udp_transport_->disconnect();
    } catch (...) {}
  }

  // ── Members ───────────────────────────────────────────────────────────

  // Parameters
  std::string robot_ip_, username_, password_, torque_topic_;
  int dof_, period_ms_;

  // ROS2
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_feedback_;

  // Kortex
  std::unique_ptr<k_api::TransportClientTcp>                          tcp_transport_;
  std::unique_ptr<k_api::TransportClientUdp>                          udp_transport_;
  std::unique_ptr<k_api::RouterClient>                                tcp_router_;
  std::unique_ptr<k_api::RouterClient>                                udp_router_;
  std::unique_ptr<k_api::SessionManager>                              session_manager_;
  std::unique_ptr<k_api::Base::BaseClient>                            base_;
  std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient>                base_cyclic_;
  std::unique_ptr<k_api::ActuatorConfig::ActuatorConfigClient>        actuator_cfg_;

  k_api::BaseCyclic::Command          base_command_;
  k_api::Base::ServoingModeInformation prev_servoing_mode_;

  // Shared state
  std::mutex           torque_mutex_;
  std::vector<double>  desired_torques_;
  std::atomic<bool>    running_{false};
  std::thread          cyclic_thread_;
};

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KortexTorqueBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}