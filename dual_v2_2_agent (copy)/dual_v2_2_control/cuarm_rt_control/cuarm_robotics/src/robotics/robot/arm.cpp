#include "arm.h"

Arm::Arm(const VectorXf gear_ratio, const VectorXf link_lengths){
    gear_ratio_ = gear_ratio;
    link_lengths_ = link_lengths;
}

VectorXd Arm::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const {
    return X;
}

bool Arm::geometricIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const {
    return false;
}

bool Arm::geometricNullSpaceControl(const double theta, const Matrix3d& R, const Vector3d& P, const Vector3d& P4, VectorXd& res) const {
    return false;
}

double Arm::boundJointAngle(double q){
    if (q < -M_PI)
        q += 2*M_PI;
    if (q > M_PI)
        q -= 2*M_PI;
    return q;
}