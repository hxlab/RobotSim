#include "controller/gripper_control.hpp"

namespace controller {

    void GazeboGripperController::on_configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node){
        // no action server configuration needed
    }

    void GazeboGripperController::on_activate(hardware_interface::LoanedCommandInterface& command_interfaces){
        gripper_ = &command_interfaces;
    }

    void GazeboGripperController::on_deactivate(){
        // nothing to deactivate
    }

    void GazeboGripperController::open(){
        desired_effort_ = -20.0;
    }

    void GazeboGripperController::close(){
        desired_effort_ = 20.0;
    }

    void GazeboGripperController::update(){
        gripper_->set_value(desired_effort_);
    }

    void FrankaGripperController::on_configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node){
        namespace_ = node->get_namespace();
        if (namespace_ == "/") {
            namespace_ = "";  // avoid double backslashes in action names
        }
        gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
        node, fmt::format("{}/franka_gripper/grasp", namespace_));
        gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
            node, fmt::format("{}/franka_gripper/move", namespace_));
        gripper_stop_client_ = node->create_client<std_srvs::srv::Trigger>(
            fmt::format("{}/franka_gripper/stop", namespace_));
    }

    void FrankaGripperController::on_activate(hardware_interface::LoanedCommandInterface& command_interfaces){
        // no loaned command interface
    }

    void FrankaGripperController::on_deactivate(){
        // Disable gripper
        if (gripper_stop_client_->service_is_ready()) {
            std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = gripper_stop_client_->async_send_request(request);
            // TODO: should print result
        }
    }

    void FrankaGripperController::open(){
        // define open gripper goal
        franka_msgs::action::Move::Goal move_goal;
        move_goal.width = 0.08;
        move_goal.speed = 0.2;

        std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>>
            move_goal_handle = gripper_move_action_client_->async_send_goal(move_goal);
    }

    void FrankaGripperController::close(){
        // TODO: should adjust the grasp goal parameters based on the object size or user input
        // NOTE: This is currently just copy+paste from the gripper example

        // Arbitrary Goal - grasp a "Magic Marker"
        // 15 mm anticipated width (diameter of cylinder)
        // bic pen: 0.008 < 0.015 - 0.005  is a fail
        // mini flashlight 0.30 > 0.015 + 0.010 is a fail
        franka_msgs::action::Grasp::Goal grasp_goal;
        grasp_goal.width = 0.015;
        grasp_goal.speed = 0.05;
        grasp_goal.force = 100.0;
        grasp_goal.epsilon.inner = 0.005;  // 10mm or less == fail !
        grasp_goal.epsilon.outer = 0.010;  // 25mm or more == fail !

        std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>>
            grasp_goal_handle =
                gripper_grasp_action_client_->async_send_goal(grasp_goal);
    }

    void FrankaGripperController::update(){
        // no update needed
    }

} // namespace controller