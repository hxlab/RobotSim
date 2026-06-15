// Copyright (c) 2023 Franka Robotics GmbH
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

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_state.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <std_msgs/msg/float64.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint position example controller moves in a periodic movement.
 */
class JointPositionExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
 // External command members
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_subscription_;
  std::vector<double> target_joint_positions_;
  bool has_new_target_;
  std::mutex target_mutex_;
  void jointPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  std::string arm_id_;
  bool is_gazebo_{false};
  std::string robot_description_;
  const int num_joints = 7;
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
  double elapsed_time_ = 0.0;
  double initial_robot_time_ = 0.0;
  double robot_time_ = 0.0;
  double trajectory_period_ = 0.001;
  bool initialization_flag_{true};
  rclcpp::Time start_time_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;

  // Gripper control members
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_subscription_;
  double target_gripper_width_;
  bool has_new_gripper_target_;
  void gripperCallback(const std_msgs::msg::Float64::SharedPtr msg);
};

}  // namespace franka_example_controllers
