#ifndef CUARM_V3_H
#define CUARM_V3_H

#include "robotics/robot/arm.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"

class CuarmV3: public Arm{
    public:
        CuarmV3(const VectorXf gear_ratio, const VectorXf link_lengths);
        VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const override;
    private:
        int joint_size_ = 7;
};
#endif