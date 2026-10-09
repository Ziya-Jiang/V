#ifndef EUARM_H
#define EUARM_H

#include "robotics/robot/arm.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"

class Euarm: public Arm{
    public:
        Euarm(const VectorXf gear_ratio, const VectorXf link_lengths);
        VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const override;
    private:
        int joint_size_ = 6;
};
#endif