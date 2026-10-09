#include "euarm.h"


Euarm::Euarm(const VectorXf gear_ratio, const VectorXf link_lengths): Arm(gear_ratio, link_lengths){

}

VectorXd Euarm::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const {
    return X;
}