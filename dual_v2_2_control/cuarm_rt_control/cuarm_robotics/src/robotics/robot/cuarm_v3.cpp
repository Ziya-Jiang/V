#include "cuarm_v3.h"

CuarmV3::CuarmV3(const VectorXf gear_ratio, const VectorXf link_lengths): Arm(gear_ratio, link_lengths){
    
}

/** Convert between motor and joint.
 * 
 * \param X Vector of data
 * \param convert_to The data type that is going to be transformed
 * \param is_tau Whether this is a torque conversion
 */
VectorXd CuarmV3::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const {
    VectorXf gear_ratio(gear_ratio_);
    Matrix2d A;
    if (!is_tau){
        A << 1.0, 0.0, -1.0, -1.0; 
    }else if (is_tau){
        A << -0.5, -0.5, 0.0, 1.0; 
    }
    VectorXd X1(X.size());
    X1[0] = (convert_to == ConvertType::kMotor)? X[0]*gear_ratio[0] : X[0]/gear_ratio[0];
    for (int i=1; i<X.size(); i+=2){
        Vector2d u1{X[i], X[i+1]};
        Array2d B = Array2d::Ones();
        if (convert_to == ConvertType::kJoint && !is_tau){
            u1[0] /= gear_ratio[i];
            u1[1] /= gear_ratio[i+1];
        }else if (convert_to == ConvertType::kJoint && is_tau){
            u1[0] /= gear_ratio[i];
            u1[1] /= gear_ratio[i+1];
        }else if (convert_to == ConvertType::kMotor && !is_tau){
            B[0] *= gear_ratio[i];
            B[1] *= gear_ratio[i+1];
        }else if (convert_to == ConvertType::kMotor && is_tau){
            B[0] *= gear_ratio[i];
            B[1] *= gear_ratio[i+1];
        }
        Vector2d u2 = A*u1;
        Array2d u3 = u2.array()*B;
        X1[i] = u3[0];
        X1[i+1] = u3[1];
    }
    return X1;
}
