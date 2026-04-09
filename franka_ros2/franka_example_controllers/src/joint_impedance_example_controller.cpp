#include <franka_example_controllers/joint_impedance_example_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}


controller_interface::return_type JointImpedanceExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  updateJointStates();

  Vector7d tau_d;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    tau_d = d_gains_.cwiseProduct(target_velocity_ - dq_);
  }

  //RCLCPP_INFO(get_node()->get_logger(),"D gains: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",d_gains_(0), d_gains_(1), d_gains_(2),d_gains_(3), d_gains_(4), d_gains_(5), d_gains_(6));

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  if (k_gains.empty() || k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(),
      "k_gains must be set with exactly %d values", num_joints);
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty() || d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(),
      "d_gains must be set with exactly %d values", num_joints);
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_joints; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_gains_(i) = d_gains.at(i);
  }

  dq_filtered_.setZero();
  target_velocity_.setZero();
  q_goal_.setZero();

  // Subscribe to velocity commands from the admittance node
  velocity_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/target_joint_velocities", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (static_cast<int>(msg->data.size()) != num_joints) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
          *get_node()->get_clock(), 1000,
          "Received velocity command with wrong size: %zu (expected %d)",
          msg->data.size(), num_joints);
        return;
      }
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      for (int i = 0; i < num_joints; ++i) {
        target_velocity_(i) = msg->data[i];
      }
      last_command_time_ = get_node()->now();
    });

  RCLCPP_INFO(get_node()->get_logger(),
    "Subscribed to /target_joint_velocities");

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  // Seed the integrated target at the current position so there's no
  // jump when the first velocity command arrives
  q_goal_ = q_;
  target_velocity_.setZero();
  start_time_ = this->get_node()->now();
  last_command_time_ = start_time_;
  return CallbackReturn::SUCCESS;
}

void JointImpedanceExampleController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerInterface)