#include "cuarm_v2.h"

#include <iostream>
#include "robotics/robotics_base.h"

CuarmV2::CuarmV2(const VectorXf gear_ratio, const VectorXf link_lengths): Arm(gear_ratio, link_lengths){
    
}

/** Convert between motor and joint.
 * 
 * \param v Vector of data
 * \param convert_to The data type that is going to be transformed
 * \param is_tau Whether this is a torque conversion
 */
VectorXd CuarmV2::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const {
    Eigen::Matrix<double,2,2> A,B,C;
    A << -0.5, 0.5, -0.5, -0.5; // Differential to joint ratio
    B << 1, 0, 0, 1;            // Motor to differential gear ratio
    if (convert_to == ConvertType::kJoint && !is_tau){
        C = A * B;
    }else if (convert_to == ConvertType::kJoint && is_tau){
        C = (A * B).transpose().inverse();
    }else if (convert_to == ConvertType::kMotor && !is_tau){
        C = (A * B).inverse();
    }else if (convert_to == ConvertType::kMotor && is_tau){
        C = (A * B).transpose();
    }

    VectorXf gear_ratio(gear_ratio_);
    if ((convert_to == ConvertType::kMotor) ^ (is_tau)){
        for (int i=0; i<gear_ratio.size(); i++){
            gear_ratio[i] = 1/gear_ratio[i];
        }
    }
    
    VectorXd X1(X.size());
    X1[0] = X[0]*gear_ratio[0];
    for (int i=1; i<X.size(); i+=2){
        Vector2d u1{X[i], X[i+1]};
        Vector2d u2 = C*u1*gear_ratio[i];
        X1[i] = u2[0];
        X1[i+1] = u2[1];
    }
    return X1;
}


/** Calculate inverse kinematics with geomatric method. The robot is reduced to 6 DOF with last joint fixed.
 * 
 * \param R     target 3x3 orientation matrix from base to tool frame.
 * \param P     target 3x1 translation vector from base to tool frame.
 * \param res   input current joint position, return the calculated joint postion.
 * \returns     state of IK
 */
bool CuarmV2::geometricIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const {
    float L1 = 0, L3 = 0, L5 = 0, L7 = 0;
    if (link_lengths_.size() == joint_size_){
        L1 = link_lengths_[0], L3 = link_lengths_[2], L5 = link_lengths_[4], L7 = link_lengths_[6];
    }else{
        std::cout <<"geometricIK only available for cuarm now.\n";
    }
    double eps = 1e-6;
    VectorXd q0 = res;
    Eigen::Matrix<double, 6, 8> qs; 
    
    // Convert from tool to end-effector frame.
    Vector3d P0 = P - R.col(2)*L7;
    Matrix3d R0 = R*RoboticsBase::MDH_R(M_PI/2, q0[6]).transpose();
    Matrix4d bTe = Matrix4d::Identity();
    bTe.block<3,3>(0,0) = R0;
    bTe.block<3,1>(0,3) = P0;

    Matrix4d T60 = bTe.completeOrthogonalDecomposition().pseudoInverse(); // Transformation matrix from frame 6 to 0.
    Matrix4d T16 = bTe; // Transformation matrix from frame 1 to 6.
    T16(2, 3) -= L1; // Cuarm's first link is always the same direction as base z-axis.
    Matrix4d T61 = T16.completeOrthogonalDecomposition().pseudoInverse();  // Transformation matrix from frame 6 to 1.
    double L61 = T61.col(3).head(3).norm();

    // Joint 4 can be found intuitively by Cos Rule.
    double a = (L3*L3 + L5*L5 - L61*L61) / 2.0 / L3 / L5;
    if (a > 1 || a < -1){
        return false;
    }
    double q41 = M_PI - acos(a);
    double q42 = M_PI + acos(a);

    // Let's form a plane m using L3 and L5, which make joint 3,4,5,6,7 line on the same place.
    // p1' is the projection point of joint 1 on plane m. P41 is the distance from joint 4 to p1'. 
    // P61 is the distance from joint 6 to p1'. aP61L5 is the included angle between P61 and L5.
    if (L3*L3 + eps < T61(2,3)*T61(2,3)){
        return false;
    }
    double P41 = sqrt(L3*L3 - T61(2,3)*T61(2,3));

    double P61 = sqrt(T61(0,3)*T61(0,3) + T61(1,3)*T61(1,3));
    double b = (L5*L5 + P61*P61 - P41*P41) / 2.0 / L5 / P61;
    if (b > 1 || b < -1){
        return false;
    }
    double aP61L5 = -fabs(acos(b));
    double q61 = atan2(T61(0,3), T61(1,3)) + aP61L5;
    double q62 = atan2(T61(0,3), T61(1,3)) - aP61L5;
    double q51 = -atan2(T61(2,3), P61 * sin(aP61L5));
    double q52 = +atan2(T61(2,3), P61 * sin(aP61L5));

    //Utilizing components of T03, joint 1,2,3 can be found.
    Matrix3d R36, R30;
    double q11, q21, q31, q12, q22, q32;
    Eigen::Matrix<double, 3,4> q456;
    q456 << q41,    q41,    q42,    q42,
            q51, q52+M_PI,    q52, q51+M_PI,
            q62,    q61,    q61,    q62;
    for (int i=0; i<4; i++){
        R36 = RoboticsBase::MDH_R(-M_PI/2, q456(0,i)) * RoboticsBase::MDH_R(+M_PI/2, q456(1,i)) * RoboticsBase::MDH_R(-M_PI/2, q456(2,i));
        R30 = R36 * T60.block<3,3>(0,0);
        q11 = atan2(+R30(2,1), +R30(2,0));
        q21 = atan2(+sqrt(R30(2,0)*R30(2,0) + R30(2,1)*R30(2,1)), +R30(2,2));
        q31 = atan2(+R30(1,2), -R30(0,2));
        qs.col(i*2) << q11, q21, q31, q456.col(i);

        q11 = atan2(-R30(2,1), -R30(2,0));
        q21 = atan2(-sqrt(R30(2,0)*R30(2,0) + R30(2,1)*R30(2,1)), +R30(2,2));
        q31 = atan2(-R30(1,2), +R30(0,2));
        qs.col(i*2+1) << q12, q22, q32, q456.col(i);
    }
    
    //The q with a smallest norm distance between q0 is chosen as the solution.
    double min_err = 1e5, err;
    int min_i;
    for (int i=0; i<8; i++){
        for (int j=0; j<6; j++){
            qs(j,i) = boundJointAngle(qs(j,i));
        }
        err = (qs.col(i) - q0.block<6,1>(0,0)).norm();
        if (err < min_err){
            min_i = i;
            min_err = err;
        }
    }
    res << qs.col(min_i), q0[6];
    return true;
}

/** Calculate null space with geomatric method. J4 always stays on the redundancy circular disk.
 * 
 * \param R     target 3x3 orientation matrix from base to tool frame.
 * \param P     target 3x1 translation vector from base to tool frame.
 * \param P4    initial 3x1 translation vector from base to joint 4 (i.e. when theta = 0).
 * \param res   input current joint position, return the calculated joint postion.
 * \returns     state of IK
 */
bool CuarmV2::geometricNullSpaceControl(const double theta, const Matrix3d& R, const Vector3d& P, const Vector3d& P4,VectorXd& res) const {
    float L1 = 0, L3 = 0, L5 = 0, L7 = 0;
    if (link_lengths_.size() == joint_size_){
        L1 = link_lengths_[0], L3 = link_lengths_[2], L5 = link_lengths_[4], L7 = link_lengths_[6];
    }else{
        std::cout <<"geometricNullSpaceControl only available for cuarm now.\n";
    }
    VectorXd q0 = res;

    Vector3d P01; P01 << 0, 0, L1;
    Vector3d P06 = P - R.col(2)*L7;
    Vector3d P16 = P06 - P01;
    double L16 = P16.norm();
    double aa = (L3*L3 + L16*L16 - L5*L5) / 2.0 / L3 / L16;
    if (aa > 1 || aa < -1){
        return false;
    }
    double aL3L16 = acos(aa);
    Vector3d P1c = (P16/L16)*L3*cos(aL3L16); //Position from joint 1 to redundancy circular disk center
    double r = L3*sin(aL3L16); //Redundancy circuit r, which is measured from joint 3 to disk center
    
    // vector a,b span the redundancy disk. When theta=0, Pc4 is aligned with vector a.
    Vector3d a, b;
    Vector3d P14 = P4 - P01;
    a = P1c-P14;
    a /= a.norm();
    b = a.cross(P1c);
    b /= b.norm();

    // P24 is the same direction as the negative J2-axis.
    // q1, q2 can be soved by R02.
    Vector3d P14_1 =  P1c - r*(cos(theta)*a + sin(theta)*b);
    Vector3d normalized_P24 = P14_1.normalized();
    double q1 = atan2(normalized_P24[1], normalized_P24[0]);
    double q2 = atan2(sqrt(normalized_P24[1]*normalized_P24[1] + normalized_P24[0]*normalized_P24[0]), normalized_P24[2]);
    Matrix4d T02 = RoboticsBase::MDH(0,0,L1,q1)*RoboticsBase::MDH(0,-M_PI/2,0,q2);

    // Since q4 is fixed, J6 position is determined only be q3.
    // q3 is solved by using P26.
    Matrix4d T20 = T02.completeOrthogonalDecomposition().pseudoInverse();
    Vector4d P06_1; P06_1 << P06, 1;
    Vector4d P26 = T20 * P06_1;
    double q4 = q0[3];
    double q3 = atan2(sin(q4)*P26[2], sin(q4)*P26[0]);
    Matrix4d T04 = T02*RoboticsBase::MDH(0,M_PI/2,L3,q3)*RoboticsBase::MDH(0,-M_PI/2, 0,q4);

    // P67 is the same direction as the negative J6 y-axis.
    Vector3d P67 = P - P06;
    Vector3d normalized_P67 = P67.normalized();
    Vector3d neg_R46_y = T04.block<3,3>(0,0).transpose() * normalized_P67;
    double q5 = atan2(neg_R46_y[2], neg_R46_y[0]);
    double q6 = atan2(sqrt(neg_R46_y[2]*neg_R46_y[2] + neg_R46_y[0]*neg_R46_y[0]), -neg_R46_y[1]);

    // q7 is solved by R67
    Matrix4d T06 = T04*RoboticsBase::MDH(0,M_PI/2,L5,q5)*RoboticsBase::MDH(0,-M_PI/2,0,q6);
    Matrix3d R67 = T06.block<3,3>(0,0).transpose() * R;
    double q7 = atan2(R67(2,0), R67(0,0));

    Eigen::Matrix<double, 2, 3> q123, q567;
    q123 <<    q1,  q2,    q3,
            q1-M_PI, -q2, q3-M_PI;
    q567 <<    q5,  q6,    q7,
            q5-M_PI, -q6, q7-M_PI;
            
    Eigen::Matrix<double, 4, 7> qs;
    double min_err = 1e5, err;
    int min_i = 0;
    Vector3d v1, v2;
    VectorXd temp_q(joint_size_);
    for (int i=0; i<2; i++){
        for (int j=0; j<2; j++){
            v1 = q123.row(i);
            v2 = q567.row(j);
            temp_q << v1, q4, v2;
            for (int k=0; k<7; k++){
                temp_q[k] = boundJointAngle(temp_q[k]);
            }
            qs.row(i*2+j) = temp_q;
            err = (temp_q - q0).norm();
            if (err < min_err){
                min_err = err;
                min_i = i*2+j;
            }
        }
    }

    //Advoid big jump.
    if (min_err > M_PI/2){
        return false;
    }
    res = qs.row(min_i);
    return true;
}
