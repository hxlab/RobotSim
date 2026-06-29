#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/loaned_state_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

#include <Eigen/Eigen>

namespace robot_model_interface
{
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    class RobotModelInterface{
        public: 
            virtual ~RobotModelInterface() = default;
            virtual void updateState(const Vector7d& q,const Vector7d& dq) = 0;

            virtual Eigen::Affine3d getPose(const std::string& frame_name) = 0;
            virtual Eigen::Matrix<double, 6, 7> getJacobian(const std::string& frame_name) = 0;
            virtual Eigen::Matrix<double, 7, 7> getMassMatrix() = 0;
            virtual Vector7d getCoriolis() = 0;
            virtual Vector7d getGravity() = 0;

            virtual std::vector<std::string> get_state_interface_names() = 0;
            virtual void assign_loaned_state_interfaces(
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) = 0;
            virtual void release_interfaces() = 0;

    };

    class RobotModelFranka : public RobotModelInterface
    {
        public: 
            RobotModelFranka(const std::string& robot_type);

            void updateState(const Vector7d& q,const Vector7d& dq) override;

            Eigen::Affine3d getPose(const std::string& frame_name) override;
            Eigen::Matrix<double, 6, 7> getJacobian(const std::string& frame_name) override;
            Eigen::Matrix<double, 7, 7> getMassMatrix() override;
            Vector7d getCoriolis() override;
            Vector7d getGravity() override;

            std::vector<std::string> get_state_interface_names() override;
            void assign_loaned_state_interfaces(
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;
            void release_interfaces() override;

        
        private:
            std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
            Vector7d q_;
            Vector7d dq_;
            const std::string k_robot_state_interface_name{"robot_state"};
            const std::string k_robot_model_interface_name{"robot_model"};

                franka::Frame stringToFrankaFrame(const std::string& frame_name) const;
    };

    class RobotModelPinocchio : public RobotModelInterface
    {
        public: 
            RobotModelPinocchio(const std::string& robot_description);

            void updateState(const Vector7d& q,const Vector7d& dq) override;

            Eigen::Affine3d getPose(const std::string& frame_name) override;
            Eigen::Matrix<double, 6, 7> getJacobian(const std::string& frame_name) override;
            Eigen::Matrix<double, 7, 7> getMassMatrix() override;
            Vector7d getCoriolis() override;
            Vector7d getGravity() override;

            std::vector<std::string> get_state_interface_names() override;
            void assign_loaned_state_interfaces(
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;
            void release_interfaces() override;

            std::string resolveFrameName(const std::string& frame_name) const;

        
        private:
            pinocchio::Model model_;
            pinocchio::Data data_;
            Vector7d q_;
            Vector7d dq_;
    };
}