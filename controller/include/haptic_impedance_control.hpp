#pragma once

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HapticImpedanceController : public controller_interface::ControllerInterface {
 public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, 
                                             const rclcpp::Duration& period) override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
    /** 
    * @brief computes the torque commands based on the impedance control law applied to end-effector pose. 
    * Calculates the wrench, W = Kp * (x_desired - x_current) + Kd * (xdot_current). 
    * Then uses the Jacobian transpose to map the wrench to joint torques, tau = J^T * W. 
    * 
    * @return Vector7d torque for each joint of the robot */
    Vector7d compute_torque_command(const Vector7d& ee_position_desired,
                                    const Vector7d& ee_position_current,
                                    const Vector7d& ee_velocity_current);

    void update_joint_states();

    Eigen::Matrix<double,6,6> stiffness_;
    Eigen::Matrix<double,6,6> damping_;

    Vector7d ee_target_position_;

    const bool k_elbow_activated_{false};
    int num_joints_{7};
    double elapsed_time_{0.0};

    std::unique_ptr<franka_semantic_components::FrankaRobotModel> model_;
    std::string robot_description_;
    std::string robot_type_;
    std::string arm_prefix_;
};