#include "controller/haptic_impedance_control.hpp"

#include <array>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace controller {

// adjustable parameters
CallbackReturn HapticImpedanceController::on_init() {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<bool>("gazebo", true);

    // Impedance control parameters
    auto_declare<double>("translational_stiffness", 1000.0);
    auto_declare<double>("rotational_stiffness", 50.0);

    // haptic device parameters
    auto_declare<double>("position_scale_x", 1.0);
    auto_declare<double>("position_scale_y", 1.0);
    auto_declare<double>("position_scale_z", 1.0);
    auto_declare<double>("height_offset", 0.3); 

    return CallbackReturn::SUCCESS;
}

// Haptic device callback functions
void HapticImpedanceController::set_goal_pose_callback(
    const geometry_msgs::msg::Pose::SharedPtr msg) {

    pose_mutex_.lock();
    position_d_ = Eigen::Vector3d(
        (-msg->position.z * position_scale_x_) + height_offset_,
        (-msg->position.x * position_scale_y_),
        (msg->position.y * position_scale_z_) + height_offset_);

    // convert haptic orientation from message to Eigen
    orientation_d_ = Eigen::Quaterniond(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);

    // took this from Steven's code
    Eigen::Quaterniond rot_1(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond rot_0(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond base_down(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    orientation_d_ = (rot_1 * orientation_d_ * rot_0 * base_down).normalized();
    pose_mutex_.unlock();
}

void HapticImpedanceController::button_callback(
    const std_msgs::msg::Int32::SharedPtr msg) {
    // Toggle the gripper state when the button is pressed
    if (msg->data == 1) {
        toggle_gripper_state();
    }
}

// Main control code
HapticImpedanceController::Vector7d
HapticImpedanceController::compute_cartesian_impedance_torque(const Eigen::Vector3d& current_position,
                                                              const Eigen::Quaterniond& current_orientation,
                                                              const Vector7d& dq) {
    // Jacobian for end-effector                                                                
    auto jacobian = robot_model_->getJacobian("end_effector");

    // end-effector velocity
    Vector6d xdot = jacobian * dq;

    // end-effector position error
    Eigen::Vector3d position_error = position_d_ - current_position;

    // end-effector orientation error
    Eigen::Quaterniond q_err = orientation_d_ * current_orientation.inverse();
    Eigen::AngleAxisd aa(q_err);
    Eigen::Vector3d orientation_error = aa.axis() * aa.angle();

    Vector6d error;
    error.head<3>() = position_error;
    error.tail<3>() = orientation_error;

    // spring-damper control law: W = Kp * error - Kd * xdot
    Vector6d wrench = Kp_ * error - Kd_ * xdot;
    Vector7d tau_task = jacobian.transpose() * wrench;

    // Coriolis compensation
    auto coriolis = robot_model_->getCoriolis();
    // Gravity compensation
    auto gravity = robot_model_->getGravity();

    return tau_task + coriolis + gravity;
}

controller_interface::return_type
HapticImpedanceController::update(
    const rclcpp::Time&,
    const rclcpp::Duration&) {
  
    // update joint states (member variables & robot model)
    update_joint_states();
    Vector7d dq;
    for (int i = 0; i < kNumJoints; i++) {
        dq(i) = joint_velocities_current_[i];
    }
    Vector7d q = Vector7d::Zero();
    for (int i = 0; i < kNumJoints; i++) {
        q(i) = joint_positions_current_[i];
    }
    robot_model_->updateState(q, dq);   // stuff breaks if we forget this

    auto coriolis = robot_model_->getCoriolis();
    auto pose = robot_model_->getPose("end_effector");
    auto jacobian = robot_model_->getJacobian("end_effector");

    Eigen::Quaterniond current_orientation;
    Eigen::Vector3d current_position;
    current_position = pose.translation();
    current_orientation = Eigen::Quaterniond(pose.linear());

    if (initialization_flag_) {
        position_d_ = current_position;
        orientation_d_ = current_orientation;
        initialization_flag_ = false;
    }

    Vector7d tau_d = compute_cartesian_impedance_torque(current_position,
                                                        current_orientation,
                                                        dq);

    for (int i = 0; i < kNumJoints; i++) {
        command_interfaces_[i].set_value(tau_d(i));
    }

    return controller_interface::return_type::OK;
}

void HapticImpedanceController::update_joint_states() {
    int offset = 0;
    if (!is_gazebo_) offset = 16;

    for (int i = 0; i < kNumJoints; i++) {
        joint_positions_current_[i] = state_interfaces_.at(offset + i).get_value();
        joint_velocities_current_[i] = state_interfaces_.at(offset + kNumJoints + i).get_value();
        joint_efforts_current_[i] = state_interfaces_.at(offset + 2 * kNumJoints + i).get_value();
    }
}

// Controller set up methods
controller_interface::InterfaceConfiguration
HapticImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (int i = 1; i <= kNumJoints; i++) {
        config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
    }

    return config;
}

controller_interface::InterfaceConfiguration
HapticImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    if (!is_gazebo_) {
        config.names = robot_model_->get_state_interface_names();
    }

    for (int i = 1; i <= kNumJoints; i++) {
        config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
    }
    for (int i = 1; i <= kNumJoints; i++) {
        config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
    }
    for (int i = 1; i <= kNumJoints; i++) {
        config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
    }

    if (!is_gazebo_) {
        for (const auto& interface : robot_model_->get_state_interface_names()) {
            config.names.push_back(interface);
        }

        config.names.push_back(arm_prefix_ + robot_type_ + "/robot_time");
    }

    return config;
}

CallbackReturn HapticImpedanceController::on_configure(const rclcpp_lifecycle::State&) {
    // ROS2 parameters
    robot_type_ = get_node()->get_parameter("robot_type").as_string();
    arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
    is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
    arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";

    // for state tracking (EE position/orientation), Jacobian, Coriolis
    if (is_gazebo_) {
        auto temp_node = std::make_shared<rclcpp::Node>("_urdf_fetcher");
        std_msgs::msg::String msg;

        rclcpp::QoS qos(1);
        qos.transient_local();

        if (!rclcpp::wait_for_message(msg, temp_node, "/robot_description",
                std::chrono::seconds(5), qos)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Timed out waiting for /robot_description topic");
            return CallbackReturn::ERROR;
        }

        robot_model_ = std::make_unique<robot_model_interface::RobotModelPinocchio>(msg.data);
        RCLCPP_INFO(get_node()->get_logger(), "Using Pinocchio model backend (simulation)");
    } 
    else {
        robot_model_ = std::make_unique<robot_model_interface::RobotModelFranka>(
            arm_prefix_ + robot_type_);
        RCLCPP_INFO(get_node()->get_logger(), "Using Franka FCI model backend (hardware)");
    }


    // Impedance control parameters
    double translational_stiffness = get_node()->get_parameter("translational_stiffness").as_double();
    double rotational_stiffness    = get_node()->get_parameter("rotational_stiffness").as_double();
    double translational_damping   = 2.0 * std::sqrt(translational_stiffness);
    double rotational_damping      = 2.0 * std::sqrt(rotational_stiffness);

    Kp_.setZero();
    Kd_.setZero();
    Kp_.topLeftCorner<3,3>()       = translational_stiffness * Eigen::Matrix3d::Identity();
    Kp_.bottomRightCorner<3,3>()   = rotational_stiffness * Eigen::Matrix3d::Identity();
    Kd_.topLeftCorner<3,3>()       = translational_damping * Eigen::Matrix3d::Identity();
    Kd_.bottomRightCorner<3,3>()   = rotational_damping * Eigen::Matrix3d::Identity();

    // Haptic device parameters
    position_scale_x_              = get_node()->get_parameter("position_scale_x").as_double();
    position_scale_y_              = get_node()->get_parameter("position_scale_y").as_double();
    position_scale_z_              = get_node()->get_parameter("position_scale_z").as_double();
    height_offset_                 = get_node()->get_parameter("height_offset").as_double();

    // Get underlying ROS2 node to create subscribers and publishers
    auto node_ptr = get_node();
    if (!node_ptr) {
        RCLCPP_ERROR(get_node()->get_logger(), "Unable to lock node ptr in on_configure");
        return CallbackReturn::ERROR;
    }

    // Haptic device subs/pubs
    haptic_pose_sub_ = node_ptr->create_subscription<geometry_msgs::msg::Pose>(
            "haptic/pose", 10,
            std::bind(&HapticImpedanceController::set_goal_pose_callback, this, std::placeholders::_1));
    button_sub_ = node_ptr->create_subscription<std_msgs::msg::Int32>(
            "haptic/buttons", 10,
            std::bind(&HapticImpedanceController::button_callback, this, std::placeholders::_1));
    haptic_force_pub_ = node_ptr->create_publisher<geometry_msgs::msg::Vector3>("haptic/force_command", 10);

    // Gripper pubs
    namespace_ = get_node()->get_namespace();
    if (namespace_ == "/") {
        namespace_ = "";  // avoid double backslashes in action names
    }
    gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      node_ptr, fmt::format("{}/franka_gripper/grasp", namespace_));
    gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
        node_ptr, fmt::format("{}/franka_gripper/move", namespace_));
    gripper_stop_client_ = node_ptr->create_client<std_srvs::srv::Trigger>(
        fmt::format("{}/franka_gripper/stop", namespace_));

    return CallbackReturn::SUCCESS;
}

CallbackReturn HapticImpedanceController::on_activate(const rclcpp_lifecycle::State&) {
    initialization_flag_ = true;
    robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn HapticImpedanceController::on_deactivate(const rclcpp_lifecycle::State&) {
    robot_model_->release_interfaces();

    // Disable gripper
    if (gripper_stop_client_->service_is_ready()) {
        std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = gripper_stop_client_->async_send_request(request);
    }
    return CallbackReturn::SUCCESS;
}

void HapticImpedanceController::toggle_gripper_state() {
    // toggle the existing gripper state between open and closed.

    if (gripper_open_) {
        close_gripper();
        gripper_open_ = false;
    } else {
        open_gripper();
        gripper_open_ = true;
    }
}

// Gripper functions
bool HapticImpedanceController::open_gripper() {
    RCLCPP_INFO(get_node()->get_logger(), "Opening the gripper - Submitting a Move Goal");

    // define open gripper goal
    franka_msgs::action::Move::Goal move_goal;
    move_goal.width = 0.08;
    move_goal.speed = 0.2;

    std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>>
        move_goal_handle = gripper_move_action_client_->async_send_goal(move_goal);

    bool ret = move_goal_handle.valid();
    if (ret) {
        RCLCPP_INFO(get_node()->get_logger(), "Submited a Move Goal");
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to submit a Move Goal");
    }
    return ret;
}

void HapticImpedanceController::close_gripper() {
    RCLCPP_INFO(get_node()->get_logger(), "Closing the gripper - Submitting a Grasp Goal");
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

    bool ret = grasp_goal_handle.valid();
    if (ret) {
    RCLCPP_INFO(get_node()->get_logger(), "Submitted a Grasp Goal");
    } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to submit a Grasp Goal");
    }
}

} // namespace controller

PLUGINLIB_EXPORT_CLASS(
    controller::HapticImpedanceController,
    controller_interface::ControllerInterface)