#ifndef ROBOTICS_LIBRARY_H
#define ROBOTICS_LIBRARY_H

#include "robot_platform_utils/cpp/include/eigen_vector.h"

class RoboticsLibrary{
    friend class RoboticsBase;
    public:
        virtual ~RoboticsLibrary() = default;
        //Getters
        virtual MatrixX7d FK(const VectorXd& joint_pos, Vector7d* ee_pose=nullptr, Vector7d* tool_pose=nullptr) = 0;
        virtual Matrix4d FK(const VectorXd& joint_pos, int joint_id) = 0;
        virtual bool IK(const Matrix3d& R, const Vector3d& P, VectorXd& res) = 0;
        virtual Vector6d calPoseDifference(const Matrix3d& R1, const Vector3d& P1, const Matrix3d& R2, const Vector3d& P2) = 0;
        virtual VectorXd ID(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc) = 0;
        virtual VectorXd IDWithExternalForces(const VectorXd& joint_pos,const VectorXd&  joint_vel,const VectorXd& joint_acc) = 0;
        virtual Matrix6Xd getJointJacobian(const VectorXd& joint_pos, int joint_id) = 0;
        virtual void getToolPose(Vector3d* pos, Eigen::Quaterniond* rot) = 0;
        virtual void updateToolFrame(const Vector3d& tool_offset) = 0;
        virtual VectorXd getJointLowerLimit() = 0;
        virtual VectorXd getJointUpperLimit() = 0;
        //Setters
        virtual void setExternalForce(const Vector3d& force_linear, const Vector3d& force_angular) = 0;
    protected:
        int joint_size_;
        Eigen::ArrayXf link_lengths_;
        Eigen::Array<bool, Eigen::Dynamic, 1> joint_revolute_;// true if it is a revolute joint;
        VectorXd joint_lower_limit_;
        VectorXd joint_upper_limit_;
};

#endif