#ifndef PINOCCHIO_WRAPPER_H
#define PINOCCHIO_WRAPPER_H

#include <string>
#include <memory>
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robotics/robotics_library/robotics_library.h"

class PinocchioWrapper: public RoboticsLibrary{
    public:
        PinocchioWrapper(const std::string& urdf_file, const std::vector<std::string>& joint_names, const std::string& ee_link_name, const Vector3d& tool_offset);
        MatrixX7d FK(const VectorXd& joint_pos, Vector7d* ee_pose=nullptr, Vector7d* tool_pose=nullptr) override;
        Matrix4d FK(const VectorXd& joint_pos, int joint_id) override;
        bool IK(const Matrix3d& R, const Vector3d& P, VectorXd& res) override;
        Vector6d calPoseDifference(const Matrix3d& R1, const Vector3d& P1, const Matrix3d& R2, const Vector3d& P2) override;
        VectorXd ID(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc) override;
        VectorXd IDWithExternalForces(const VectorXd& joint_pos,const VectorXd&  joint_vel,const VectorXd& joint_acc) override;
        Matrix6Xd getJointJacobian(const VectorXd& joint_pos, int joint_id) override;
        void getToolPose(Vector3d* pos, Eigen::Quaterniond* rot) override;
        void updateToolFrame(const Vector3d& tool_offset) override;
        VectorXd getJointLowerLimit() override;
        VectorXd getJointUpperLimit() override;

        //Setters
        void setExternalForce(const Vector3d& force_linear, const Vector3d& force_angular) override;
    private:
        // Hiding Pinocchio variables inside this forward-declared struct to
        // isolate Pinocchio's massive header size and template metaprogramming
        struct Impl;
        std::unique_ptr<Impl> pimpl_;
};

#endif