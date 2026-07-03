#pragma once

#include <mutex>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>

#include <fmt/format.h>

#include <controller_interface/controller_interface.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/wait_for_message.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace controller {

    class GripperController {
        public:
            virtual ~GripperController() = default;

            virtual void on_configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;
            virtual void on_activate(hardware_interface::LoanedCommandInterface& command_interfaces) = 0;
            virtual void on_deactivate() = 0;
            virtual void open() = 0;
            virtual void close() = 0;
            virtual void update() = 0;
    };

    class GazeboGripperController : public GripperController {
        public:
            void on_configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;
            void on_activate(hardware_interface::LoanedCommandInterface& command_interfaces) override;
            void on_deactivate() override;
            void open() override;
            void close() override;
            void update() override;
        private:
            hardware_interface::LoanedCommandInterface* gripper_;
            double desired_effort_ = -20.0;
    };

    class FrankaGripperController : public GripperController {
        public:
            void on_configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;
            void on_activate(hardware_interface::LoanedCommandInterface& command_interfaces) override;
            void on_deactivate() override; 
            void open() override;
            void close() override;
            void update() override;
        private:
            std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Grasp>> gripper_grasp_action_client_;
            std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Move>> gripper_move_action_client_;
            std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> gripper_stop_client_;
            std::string namespace_;
    };

} // namespace controller