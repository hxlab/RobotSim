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
        auto_declare<double>("translational_stiffness", 2000.0);
        auto_declare<double>("rotational_stiffness", 150.0);

        // haptic device parameters
        auto_declare<double>("position_scale_x", 3.0);
        auto_declare<double>("position_scale_y", 2.0);
        auto_declare<double>("position_scale_z", 1.0);
        auto_declare<double>("height_offset",    0.3);

        return CallbackReturn::SUCCESS;
    }

    // Haptic device callback functions
    void HapticImpedanceController::set_goal_pose_callback(
        const geometry_msgs::msg::Pose::SharedPtr msg) {

        pose_mutex_.lock();
        Eigen::Vector3d prev_position_d = position_d_;
        Eigen::Quaterniond prev_orientation_d = orientation_d_;

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

        velocity_d_ = Eigen::Vector3d::Zero();
        angular_velocity_d_ = Eigen::Vector3d::Zero();
        velocity_d_ = (position_d_ - prev_position_d) / 0.001; // 1ms update rate
        Eigen::Quaterniond delta_q = orientation_d_ * prev_orientation_d.inverse();
        Eigen::AngleAxisd aa(delta_q);
        Eigen::Vector3d angular_velocity_d_ = aa.axis() * aa.angle() / 0.001;

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
                                                                const Vector7d& q,
                                                                const Vector7d& dq) {
        // Jacobian for end-effector                                                                
        auto jacobian = robot_model_->getJacobian("end_effector");

        // end-effector velocity
        Vector6d xdot = jacobian * dq;

        // end-effector velocity error
        // Vector6d xdot_error;
        // xdot_error.head<3>() = velocity_d_ - xdot.head<3>();
        // xdot_error.tail<3>() = angular_velocity_d_ - xdot.tail<3>();

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
        Eigen::Matrix<double, 7, 6> jacobian_transpose(jacobian.transpose());
        Vector6d wrench = Kp_ * error - Kd_ * xdot;
        Vector7d tau_task = jacobian_transpose * wrench;

        // Coriolis compensation
        auto coriolis = robot_model_->getCoriolis();
        // Gravity compensation
        auto gravity = robot_model_->getGravity();

        // null space stuff
        // kinematic pseuoinverse
        Eigen::Matrix<double, 6, 7> jacobian_transpose_pinv = pseudo_inverse(jacobian_transpose);

        Vector7d dqe;
        Vector7d qe;

        qe << q_d_nullspace_ - q;
        qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
        dqe << dq;
        dqe.head(1) << dqe.head(1) * 2.0 * sqrt(joint1_nullspace_stiffness_);
        Vector7d tau_nullspace;
        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                            jacobian_transpose * jacobian_transpose_pinv) *
                            (nullspace_stiffness_ * qe -
                                (2.0 * sqrt(nullspace_stiffness_)) * dqe);

        return tau_task + tau_nullspace + coriolis + gravity;
    }

    HapticImpedanceController::Vector7d 
    HapticImpedanceController::saturate_torque_rate(const HapticImpedanceController::Vector7d& tau_d_calculated,
                                                    const HapticImpedanceController::Vector7d& tau_J_d) {
        // from https://github.com/rail-berkeley/serl_franka_controllers/blob/main/src/cartesian_impedance_controller.cpp
        Vector7d tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] =
                tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }

    Eigen::Matrix<double, 6, 7> HapticImpedanceController::pseudo_inverse(const Eigen::Matrix<double, 7, 6>& J) {
        // from https://github.com/rail-berkeley/serl_franka_controllers/blob/main/src/cartesian_impedance_controller.cpp
        double lambda = 0.2;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
        Eigen::MatrixXd S_ = J;  // copying the dimensions of J, its content is not needed.
        S_.setZero();

        for (int i = 0; i < sing_vals_.size(); i++)
            S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda * lambda);

        return Eigen::Matrix<double, 6, 7>(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
    }

    controller_interface::return_type
    HapticImpedanceController::update(
        const rclcpp::Time&,
        const rclcpp::Duration&) {
    
        // update joint states (member variables & robot model)
        update_joint_states();
        Vector7d dq;
        for (int i = 0; i < num_joints_; i++) {
            dq(i) = joint_velocities_current_[i];
        }
        Vector7d q = Vector7d::Zero();
        for (int i = 0; i < num_joints_; i++) {
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

        // NOTE: it's a bit redundant to send EE pose and joint positions, but we get q in update() anyway, and need it in compute_cartesian_impedance_torque() for nullspace control
        Vector7d tau_d = compute_cartesian_impedance_torque(current_position,       // current end-effector position
                                                            current_orientation,    
                                                            q,                      // joint positions
                                                            dq);                    // joint velocities

        Vector7d tau_d_saturated = saturate_torque_rate(tau_d, tau_d_previous_);

        for (int i = 0; i < num_joints_; i++) {
            command_interfaces_[i].set_value(tau_d_saturated(i));
            tau_d_previous_(i) = tau_d_saturated(i);
        }

        if (is_gazebo_) {
            gripper_->update();
        }

        return controller_interface::return_type::OK;
    }

    void HapticImpedanceController::update_joint_states() {
        int offset = 0;
        if (!is_gazebo_) offset = 16;

        for (int i = 0; i < num_joints_; i++) {
            joint_positions_current_[i] = state_interfaces_.at(offset + i).get_value();
            joint_velocities_current_[i] = state_interfaces_.at(offset + num_joints_ + i).get_value();
            joint_efforts_current_[i] = state_interfaces_.at(offset + 2 * num_joints_ + i).get_value();
        }
    }

    // Controller set up methods
    controller_interface::InterfaceConfiguration
    HapticImpedanceController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (int i = 1; i <= num_joints_; i++) {
            config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
        }

        // Add finger command interface if using gazebo
        if(is_gazebo_){
            config.names.push_back(arm_prefix_ + robot_type_ + "_finger_joint" + std::to_string(1) + "/effort");
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

        for (int i = 1; i <= num_joints_; i++) {
            config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
        }
        for (int i = 1; i <= num_joints_; i++) {
            config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
        }
        for (int i = 1; i <= num_joints_; i++) {
            config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
        }

        if (!is_gazebo_) {
            for (const auto& interface : robot_model_->get_state_interface_names()) {
                config.names.push_back(interface);
            }

            config.names.push_back(arm_prefix_ + robot_type_ + "/robot_time");
        }

        // Add finger command interface if using gazebo
        if(is_gazebo_){
            config.names.push_back(arm_prefix_ + robot_type_ + "_finger_joint" + std::to_string(1) + "/effort");
        }

        return config;
    }

    CallbackReturn HapticImpedanceController::on_configure(const rclcpp_lifecycle::State&) {
        // ROS2 parameters
        robot_type_ = get_node()->get_parameter("robot_type").as_string();
        arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
        is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
        arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";

        if (is_gazebo_) {
            // for state tracking (EE position/orientation), Jacobian, Coriolis
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

            // Gripper initialization
            gripper_ = std::make_unique<GazeboGripperController>();
            RCLCPP_INFO(get_node()->get_logger(), "Using Gazebo gripper controller (simulation)");
        } 
        else {
            // for state tracking (EE position/orientation), Jacobian, Coriolis
            robot_model_ = std::make_unique<robot_model_interface::RobotModelFranka>(
                arm_prefix_ + robot_type_);
            RCLCPP_INFO(get_node()->get_logger(), "Using Franka FCI model backend (hardware)");

            // Gripper initialization
            gripper_ = std::make_unique<FrankaGripperController>();
            RCLCPP_INFO(get_node()->get_logger(), "Using Franka FCI gripper controller (hardware)");
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
        haptic_force_pub_ = node_ptr->create_publisher<geometry_msgs::msg::Vector3>(
                "haptic/force_command", 10);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HapticImpedanceController::on_activate(const rclcpp_lifecycle::State&) {
        initialization_flag_ = true;

        robot_model_->assign_loaned_state_interfaces(state_interfaces_);
        gripper_->on_activate(command_interfaces_[num_joints_]);

        // initialize the nullspace variable to the current joint positions
        for (int i = 0; i < num_joints_; i++) {
            q_d_nullspace_(i) = joint_positions_current_[i];
        }
        // initialize the previous torque variable to the current joint torques
        for (int i = 0; i < num_joints_; i++) {
            tau_d_previous_(i) = joint_efforts_current_[i];
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HapticImpedanceController::on_deactivate(const rclcpp_lifecycle::State&) {
        robot_model_->release_interfaces();
        gripper_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    void HapticImpedanceController::toggle_gripper_state() {
        // toggle the existing gripper state between open and closed.
        RCLCPP_INFO(get_node()->get_logger(), "Button callback received");

        // debounce
        auto now = get_node()->get_clock()->now();
        double elapsed_time = (now - last_toggle_time_).seconds();
        RCLCPP_INFO(get_node()->get_logger(), "Elapsed time since last toggle: %f seconds", elapsed_time);
        if (elapsed_time < debounce_period_) {
            RCLCPP_INFO(get_node()->get_logger(), "Debounce period not elapsed, ignoring button press.");
            return;
        }
        last_toggle_time_ = now;

        if (gripper_open_) {
            gripper_->close();
            gripper_open_ = false;
        } else {
            gripper_->open();
            gripper_open_ = true;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Gripper state: %s", gripper_open_ ? "true" : "false");
    }

} // namespace controller

PLUGINLIB_EXPORT_CLASS(
    controller::HapticImpedanceController,
    controller_interface::ControllerInterface)