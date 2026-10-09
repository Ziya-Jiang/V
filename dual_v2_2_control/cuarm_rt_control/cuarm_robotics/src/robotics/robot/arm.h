#ifndef ARM_H
#define ARM_H

#include "robot_platform_utils/cpp/include/eigen_vector.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846  /* pi */
#endif
enum class ConvertType{kMotor, kJoint, kNone};
class Arm{
    public:
        Arm(const VectorXf gear_ratio, const VectorXf link_lengths);
        virtual ~Arm() = default;
        virtual VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const;
        virtual bool geometricIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const;
        virtual bool geometricNullSpaceControl(const double theta, const Matrix3d& R, const Vector3d& P, const Vector3d& P4, VectorXd& res) const;
        
        //static functions
        static double boundJointAngle(double q);
    protected:
        VectorXf gear_ratio_;
        VectorXf link_lengths_;
};

#endif