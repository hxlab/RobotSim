#pragma once

#include <Eigen/Dense>
#include <mutex>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>

#include <fmt/format.h>

#include <controller_interface/controller_interface.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/wait_for_message.hpp>

#include "controller/robot_interface.hpp"
#include "controller/gripper_control.hpp"

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace controller {

    class HapticImpedanceController : public controller_interface::ControllerInterface {
        public:
            using Vector7d = Eigen::Matrix<double, 7, 1>;
            using Vector6d = Eigen::Matrix<double, 6, 1>;
            using Matrix6d = Eigen::Matrix<double, 6, 6>;

            [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::return_type update(
                const rclcpp::Time& time,
                const rclcpp::Duration& period) override;

            CallbackReturn on_init() override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        private:
            void update_joint_states();

            /** 
            * @brief computes the torque commands based on the impedance control law applied to end-effector pose. 
            * Calculates the wrench, W = Kp * (x_desired - x_current) + Kd * (xdot_current). 
            * Then uses the Jacobian transpose to map the wrench to joint torques, tau = J^T * W. 
            * 
            * @return Vector7d torque for each joint of the robot */
            Vector7d compute_cartesian_impedance_torque(
                const Eigen::Vector3d& current_position,
                const Eigen::Quaterniond& current_orientation,
                const Vector7d& q,
                const Vector7d& dq);

            Vector7d saturate_torque_rate(
                const Vector7d& tau_d_calculated,
                const Vector7d& tau_J_d);

            Eigen::Matrix<double, 6, 7> pseudo_inverse(const Eigen::Matrix<double, 7, 6>& J);

            // Haptic device methods
            void set_goal_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
            void button_callback(const std_msgs::msg::Int32::SharedPtr msg);

            // Gripper methods
            void toggle_gripper_state();

            static constexpr int num_joints_ = 7;

            // Desired end-effector pose (from haptic device)
            Eigen::Vector3d position_d_;
            Eigen::Quaterniond orientation_d_;

            Eigen::Vector3d velocity_d_;
            Eigen::Vector3d angular_velocity_d_;

            // Haptic pose position scaling
            double position_scale_x_;
            double position_scale_y_;
            double position_scale_z_;
            double height_offset_;

            // Mutex for position and orientation updates
            std::mutex pose_mutex_;

            // Impedance control gains
            Matrix6d Kp_;
            Matrix6d Kd_;

            // Other control variables
            double nullspace_stiffness_{20.0};
            double nullspace_stiffness_target_{20.0};
            double joint1_nullspace_stiffness_{20.0};
            double joint1_nullspace_stiffness_target_{20.0};
            const double delta_tau_max_{1.0};
            Vector7d q_d_nullspace_;
            Vector7d tau_d_previous_;
            
            // State variables
            std::vector<double> joint_positions_current_{0,-0.785,0,-2.356,0,1.57,0.785};
            std::vector<double> joint_velocities_current_{0,0,0,0,0,0,0};
            std::vector<double> joint_efforts_current_{0,0,0,0,0,0,0};

            // Gripper parameters
            std::unique_ptr<GripperController> gripper_;
            bool gripper_open_{false};

            // gripper debounce
            rclcpp::Time last_toggle_time_{0, 0, RCL_ROS_TIME};
            double debounce_period_ = 0.3;

            // ROS2 parameters
            std::string robot_type_;
            std::string arm_prefix_;
            bool initialization_flag_{true};
            const bool k_elbow_activated_{false};
            std::unique_ptr<robot_model_interface::RobotModelInterface> robot_model_;

            bool is_gazebo_{true};

            // ROS2 subscribers & publishers
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr haptic_pose_sub_;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr button_sub_;
            rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr haptic_force_pub_;
    };

} // namespace controller