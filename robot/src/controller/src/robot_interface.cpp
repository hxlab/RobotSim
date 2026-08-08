#include "controller/robot_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <unordered_map>

namespace robot_model_interface
{
    // Franka robot model interface
    RobotModelFranka::RobotModelFranka(const std::string& robot_type)
    {
        franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
            robot_type + "/" + k_robot_model_interface_name,
            robot_type + "/" + k_robot_state_interface_name);
    }

    std::vector<std::string> RobotModelFranka::get_state_interface_names()
    {
        return franka_robot_model_->get_state_interface_names();
    }

    void RobotModelFranka::assign_loaned_state_interfaces(
        std::vector<hardware_interface::LoanedStateInterface>& state_interfaces)
    {
        franka_robot_model_->assign_loaned_state_interfaces(state_interfaces);
    }

    void RobotModelFranka::release_interfaces()
    {
        franka_robot_model_->release_interfaces();
    }

    void RobotModelFranka::updateState(const Vector7d& q,const Vector7d& dq)
    {
        q_ = q;
        dq_ = dq;
    }

    Eigen::Affine3d RobotModelFranka::getPose(const std::string& frame_name)
    {
        std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(stringToFrankaFrame(frame_name));
        return Eigen::Affine3d(Eigen::Matrix4d::Map(pose_array.data()));
    }

    Eigen::Matrix<double, 6, 7> RobotModelFranka::getJacobian(const std::string& frame_name)
    {
        std::array<double, 42> jac_array = franka_robot_model_->getZeroJacobian(stringToFrankaFrame(frame_name));
        return Eigen::Matrix<double,6,7>(Eigen::Map<Eigen::Matrix<double,6,7>>(jac_array.data()));
    }

    Eigen::Matrix<double, 7, 7> RobotModelFranka::getMassMatrix()
    {
        std::array<double, 49> mass_array = franka_robot_model_->getMassMatrix();
        return Eigen::Matrix<double,7,7>(Eigen::Map<Eigen::Matrix<double,7,7>>(mass_array.data()));
    }

    Vector7d RobotModelFranka::getCoriolis()
    {
        std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
        return Eigen::Map<Vector7d>(coriolis_array.data());
    }

    Vector7d RobotModelFranka::getGravity()
    {
        std::array<double, 7> gravity_array = franka_robot_model_->getGravityForceVector();
        return Eigen::Map<Vector7d>(gravity_array.data());
    }

    franka::Frame RobotModelFranka::stringToFrankaFrame(const std::string& frame_name) const {
        static const std::unordered_map<std::string, franka::Frame> frame_map = {
            {"joint1", franka::Frame::kJoint1},
            {"joint2", franka::Frame::kJoint2},
            {"joint3", franka::Frame::kJoint3},
            {"joint4", franka::Frame::kJoint4},
            {"joint5", franka::Frame::kJoint5},
            {"joint6", franka::Frame::kJoint6},
            {"joint7", franka::Frame::kJoint7},
            {"flange", franka::Frame::kFlange},
            {"end_effector", franka::Frame::kEndEffector}
        };
        return frame_map.at(frame_name);
    }

    // Pinocchio robot model interface
    RobotModelPinocchio::RobotModelPinocchio(const std::string& robot_description)
    {
        pinocchio::urdf::buildModelFromXML(robot_description, model_);
        std::vector<pinocchio::JointIndex> joints_to_lock;
        joints_to_lock.push_back(model_.getJointId("fr3_finger_joint1"));
        joints_to_lock.push_back(model_.getJointId("fr3_finger_joint2"));

        Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(model_.nq);

        // The URDF model has 9 joints, but we want to lock the fingers (they are not actuated through this interface)
        model_ = pinocchio::buildReducedModel(
            model_,
            joints_to_lock,
            q_ref);

        model_.gravity.linear() = Eigen::Vector3d(0, 0, -9.81);
        data_ = pinocchio::Data(model_);
    }

    void RobotModelPinocchio::updateState(const Vector7d& q,const Vector7d& dq)
    {
        q_ = q;
        dq_ = dq;
        pinocchio::computeAllTerms(model_, data_, q_, dq_);
        pinocchio::updateFramePlacements(model_, data_);
    }

    Eigen::Matrix<double, 7, 7> RobotModelPinocchio::getMassMatrix()
    {
        return Eigen::Matrix<double,7,7>(data_.M);
    }

    Vector7d RobotModelPinocchio::getCoriolis()
    {
        return data_.nle - data_.g;
    }

    Vector7d RobotModelPinocchio::getGravity()
    {
        return data_.g;
    }

    std::vector<std::string> RobotModelPinocchio::get_state_interface_names()
    {
        return {};
    }

    void RobotModelPinocchio::assign_loaned_state_interfaces(
        std::vector<hardware_interface::LoanedStateInterface>& state_interfaces)
    {
        // No state interfaces to assign for Pinocchio model
    }

    void RobotModelPinocchio::release_interfaces()
    {
        // No interfaces to release for Pinocchio model
    }

    std::string RobotModelPinocchio::resolveFrameName(const std::string& frame_name) const
    {
        static const std::unordered_map<std::string, std::string> frame_map = {
            {"joint1",       "fr3_link1"},
            {"joint2",       "fr3_link2"},
            {"joint3",       "fr3_link3"},
            {"joint4",       "fr3_link4"},
            {"joint5",       "fr3_link5"},
            {"joint6",       "fr3_link6"},
            {"joint7",       "fr3_link7"},
            {"end_effector", "fr3_link8"},
        };
        auto it = frame_map.find(frame_name);
        return (it != frame_map.end()) ? it->second : frame_name;
    }

    Eigen::Affine3d RobotModelPinocchio::getPose(const std::string& frame_name)
    {
        auto frame_id = model_.getFrameId(resolveFrameName(frame_name));
        return Eigen::Affine3d(data_.oMf[frame_id].toHomogeneousMatrix());
    }

    Eigen::Matrix<double, 6, 7> RobotModelPinocchio::getJacobian(const std::string& frame_name)
    {
        auto frame_id = model_.getFrameId(resolveFrameName(frame_name));
        Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero();
        pinocchio::getFrameJacobian(model_, data_, frame_id,
            pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    }

} // namespace robot_model_interface