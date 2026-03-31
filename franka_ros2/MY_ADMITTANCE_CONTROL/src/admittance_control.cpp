/**
 * @file admittance_control_node.cpp
 * @brief Cartesian admittance controller for the Franka FR3 robot arm.
 *
 * Pure ROS2 topic-driven controller:
 *   - Goal:  /set_goal_pose
 *   - Force: /set_force
 *
 * No haptic device, no CLI input.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class AdmittanceNode : public rclcpp::Node
{
public:
    AdmittanceNode() : Node("admittance_control_node")
    {
        // Parameters
        this->declare_parameter<double>("mass", 10.0);
        this->declare_parameter<double>("damping", 140.0);
        this->declare_parameter<double>("stiffness", 500.0);

        m_ = this->get_parameter("mass").as_double();
        d_ = this->get_parameter("damping").as_double();
        k_ = this->get_parameter("stiffness").as_double();

        // Critical damping
        d_ = 2.0 * std::sqrt(m_ * k_);

        // State init
        ee_pos_ = Eigen::Vector3d(0.4, 0.0, 0.2);
        ee_vel_ = Eigen::Vector3d::Zero();
        set_goal_ = ee_pos_;
        set_force_ = Eigen::Vector3d::Zero();

        // Subscribers
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/set_goal_pose", 10,
            std::bind(&AdmittanceNode::goalCallback, this, std::placeholders::_1));

        force_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/set_force", 10,
            std::bind(&AdmittanceNode::forceCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&AdmittanceNode::jointStateCallback, this, std::placeholders::_1));

        // Publisher
        joint_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/target_joint_velocities", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AdmittanceNode::controlLoop, this));

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Admittance Controller Running.");

        // Load robot model
        robot_model_loader_ =
            std::make_shared<robot_model_loader::RobotModelLoader>(
                shared_from_this(), "robot_description");

        kinematic_model_ = robot_model_loader_->getModel();
        kinematic_state_ =
            std::make_shared<moveit::core::RobotState>(kinematic_model_);

        joint_model_group_ =
            kinematic_model_->getJointModelGroup("fr3_arm");
    }

private:
    // Callbacks
    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        set_goal_ << msg->position.x, msg->position.y, msg->position.z;
    }

    void forceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        set_force_ << msg->x, msg->y, msg->z;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        latest_joint_state_ = *msg;
    }

    // Main control loop
    void controlLoop()
    {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 0.1) return;
        if (latest_joint_state_.name.empty()) return;

        // Update joint state
        std::vector<std::string> joints = {
            "fr3_joint1","fr3_joint2","fr3_joint3",
            "fr3_joint4","fr3_joint5","fr3_joint6","fr3_joint7"};

        for (const auto& name : joints) {
            auto it = std::find(latest_joint_state_.name.begin(),
                                latest_joint_state_.name.end(), name);
            if (it != latest_joint_state_.name.end()) {
                size_t idx = std::distance(latest_joint_state_.name.begin(), it);
                kinematic_state_->setJointPositions(name,
                    &latest_joint_state_.position[idx]);
            }
        }
        kinematic_state_->updateLinkTransforms();

        // Forward kinematics
        const auto& tf =
            kinematic_state_->getGlobalLinkTransform("fr3_hand_tcp");
        Eigen::Vector3d actual_pos = tf.translation();

        // Error
        Eigen::Vector3d error = actual_pos - set_goal_;

        // Admittance
        Eigen::Vector3d acc =
            (set_force_ - d_ * ee_vel_ - k_ * error) / m_;

        ee_vel_ += acc * dt;
        ee_pos_ += ee_vel_ * dt;

        // Jacobian
        Eigen::MatrixXd J;
        kinematic_state_->getJacobian(
            joint_model_group_,
            kinematic_state_->getLinkModel("fr3_hand_tcp"),
            Eigen::Vector3d::Zero(), J);

        // Cartesian velocity
        Eigen::VectorXd cart_vel(6);
        cart_vel << ee_vel_.x(), ee_vel_.y(), ee_vel_.z(), 0, 0, 0;

        // Damped pseudoinverse
        double lambda = 0.05;
        Eigen::MatrixXd JJT = J * J.transpose();
        Eigen::MatrixXd J_pinv =
            J.transpose() *
            (JJT + lambda * lambda *
             Eigen::MatrixXd::Identity(6,6)).inverse();

        Eigen::VectorXd q_dot = J_pinv * cart_vel;

        // Clamp
        for (int i = 0; i < 7; i++)
            q_dot[i] = std::clamp(q_dot[i], -0.5, 0.5);

        // Publish
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(7);
        for (int i = 0; i < 7; i++)
            msg.data[i] = q_dot[i];

        joint_vel_pub_->publish(msg);
    }

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // MoveIt
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;

    // State
    sensor_msgs::msg::JointState latest_joint_state_;
    Eigen::Vector3d ee_pos_, ee_vel_;
    Eigen::Vector3d set_goal_, set_force_;

    rclcpp::Time last_time_;

    // Gains
    double m_, d_, k_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdmittanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}