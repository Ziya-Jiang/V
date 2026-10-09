#ifndef CUARM_V2_H
#define CUARM_V2_H

#include "robotics/robot/arm.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"

class CuarmV2: public Arm{
    public:
        CuarmV2(const VectorXf gear_ratio, const VectorXf link_lengths);
        VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const override;
        bool geometricIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const override;
        bool geometricNullSpaceControl(const double theta, const Matrix3d& R, const Vector3d& P, const Vector3d& P4, VectorXd& res) const override;
    private:
        int joint_size_ = 7;
};
#endif