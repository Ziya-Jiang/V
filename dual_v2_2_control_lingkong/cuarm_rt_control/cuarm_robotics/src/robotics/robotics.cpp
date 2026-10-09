#include "robotics.h"
// #include <cmath>
#include <math.h>
#include <iostream>

#include "robotics/robot/cuarm_v2.h"
#include "robotics/robot/cuarm_v3.h"
#include "robotics/robot/euarm.h"

using namespace robo;

// Saturate (clamp, bound) Vector to +/- max
VectorXd saturate(const VectorXd& max,const VectorXd& a){
    VectorXd ret = a;
    for (int idx = 0; idx < a.size(); idx++){
        double sign = RoboticsBase::Sgn(ret(idx));
        if (fabs(ret(idx)) > fabs(max(idx))) {
            ret(idx) = sign * fabs(max(idx));
        }
    }
    return ret;
}

// // Saturate (clamp, bound) Force to +/- max
// Force saturate(const Force& max, const Force& a){
//     Force ret;
//     ret.linear() = saturate(max.linear(),a.linear());
//     ret.angular() = saturate(max.angular(),a.angular());
//     return ret;
// }

Robotics::Robotics(const std::string& urdf_file, const std::vector<std::string>& joint_names, const std::string& ee_link_name, 
    const Vector3d& tool_offset, int arm_i, int joint_size, const Matrix2Xf& joint_limits, const std::string& library_name)
    : RoboticsBase(library_name, urdf_file, joint_names, ee_link_name, tool_offset), arm_i_(arm_i), 
    joint_size_(joint_size), joint_limits_(joint_limits){
    filtered_joint_vel_     = ArrayXd::Zero(joint_size_);
    prev_J_inv_tr_row_      = VectorXd::Zero(joint_size_);
    joint_force_result_     = VectorXd::Zero(joint_size_);             // Copy of force result for monitoring
    joint_force_saturation_ = VectorXd::Zero(joint_size_);         // Force values to clamp at
    gravity_with_boundary_joint_saturation_ = VectorXd::Zero(joint_size_); 
}

Robotics::~Robotics(){
    delete arm_;
    arm_ = nullptr;
}

void Robotics::setArm(const std::string& name, const VectorXf gear_ratio){
    if (arm_ != nullptr)
        delete arm_;
    if (name == "CuarmV2_2"){
        arm_ = new CuarmV2(gear_ratio, link_lengths_);
    }else if (name == "CuarmV3_0"){
        arm_ = new CuarmV3(gear_ratio, link_lengths_);
    }else if (name == "Euarm"){
        arm_ = new Euarm(gear_ratio, link_lengths_);
    }else{
        // printf("%s is not defined, default arm is used.\n", name.c_str());
        arm_ = new Arm(gear_ratio, link_lengths_);
    }
}

void Robotics::setGravityGains(const VectorXd& ratio, const VectorXd& kd){
    gravity_ratio_ = ratio.array();
    gravity_kd_ = kd.array();
} 

void Robotics::setTaskImepdance(const Vector6d& tool_kp, const Vector6d& tool_kd, const Vector6d& tool_ki, float ki_ratio, const VectorXd& joint_saturation, const VectorXd& joint_torque_ratio, const VectorXd& joint_additional_kd, const VectorXd& task_null_k){
    task_imepdance_kp_ = tool_kp;
    task_imepdance_kd_ = tool_kd;
    task_imepdance_ki_ = tool_ki;
    task_imepdance_ki_ratio_ = ki_ratio;
    task_imepdance_joint_saturation_ = joint_saturation;
    task_imepdance_joint_torque_ratio_ = joint_torque_ratio;
    task_imepdance_joint_additional_kd_ = joint_additional_kd;
    task_null_k_ = task_null_k;

    tool_force_.kp_ = tool_kp;
    tool_force_.kd_ = tool_kd;
    tool_force_.ki_ = tool_ki;
    tool_force_.ki_ratio_ = ki_ratio;
    tool_force_.prev_pos_error_ = Vector3d::Zero();
    tool_force_.prev_rot_error_ = Vector3d::Zero();
    tool_force_.pos_error_i_ = Vector3d::Zero();
    tool_force_.rot_error_i_ = Vector3d::Zero();
} 
            
void Robotics::setJointImpedance(const VectorXd& joint_kp, const VectorXd& joint_kd, const VectorXd& joint_ki){
    joint_force_.kp_ = joint_kp;
    joint_force_.kd_ = joint_kd;
    joint_force_.ki_ = joint_ki;
} 

void Robotics::setGravityWithBoundary(const Vector3d& kp, const Vector3d& boundary, const VectorXd& joint_saturation){
    gravity_with_boundary_boundary_ = boundary;
    gravity_with_boundary_kp_ = kp;
    gravity_with_boundary_joint_saturation_ = joint_saturation;
}

void Robotics::setTaskAdmittance(float kp){
    task_admittance_kp_ = kp;
}

void Robotics::setJointAdmittance(float damping, const VectorXd& force_upper_bound, const VectorXd& threshold, const VectorXd& max_speed, const VectorXd& max_acc, const VectorXd& extern_for_k){
    joint_admittance_kd_ = damping;
    joint_admittance_force_upper_bound_ = force_upper_bound;
    joint_admittance_threshold_ = threshold;
    joint_admittance_max_speed_ = max_speed;
    joint_admittance_max_acceleration_ = max_acc;
    joint_admittance_extern_force_k_ = extern_for_k;
}

void Robotics::setNullAdmittance(float kp, float kd, float threshold, float saturation, int joint_index){
    null_admittance_kp_ = kp;
    null_admittance_kd_ = kd;
    null_admittance_threshold_ = threshold;
    null_admittance_saturation_ = saturation;
    null_admittance_kp_ji_ = joint_index;
}

void Robotics::setJointSaturation(const VectorXd& joint_saturation){
    joint_force_saturation_ = joint_saturation;
}

void Robotics::setJointVelLowPassFilterRatio(const VectorXd& ratio){
    vel_low_pass_filter_ratio_ = ratio.array();
}

void Robotics::setFrictionAsCoulumbWithViscous(const VectorXd& ratio, const VectorXd& fc, const VectorXd& fv){
    friction_model_ = FrictionModel::kCoulumbWithViscous;
    friction_cv_ratio_ = ratio.array();
    friction_cv_coulumb_ = fc.array();
    friction_cv_viscous_ = fv.array();
}


// Calculate the pseudo inverse using SVD. Replace the both singular and 
// "nearly singular" values to 0 before inverting the s.
MatrixX6d Robotics::getJacobianPseudoInverse(MatrixXd J){
    auto svd = J.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& s = svd.singularValues();
    MatrixXd s_inv(J.cols(), J.rows());
    s_inv.setZero();

    // float esp = pow(2,-52);
    // float k = svd.rank()*esp;
    double k = 1e-5;
    for (int i=0; i<s.size(); i++){
        s_inv(i, i) = (s(i) > k)? 1/s(i) : 0;
    }
    
    MatrixXd sss(s.size(), s.size());
    for (int i=0; i<s.size(); i++){
        sss(i,i) = s(i);
    }
    return svd.matrixV() * s_inv * svd.matrixU().transpose();
}

VectorXd Robotics::jacobianNullSpaceControl(const VectorXd& joint_pos, const VectorXd& joint_vel_tar){
    int last_joint_id = joint_size_-1;
    Matrix6Xd J = getJointJacobian(joint_pos, last_joint_id);
    MatrixXd J_inv = getJacobianPseudoInverse(J);
    MatrixXd N = (MatrixXd::Identity(joint_size_, joint_size_) - J_inv*J);
    VectorXd dv = N*joint_vel_tar;
    return dv;
}

bool Robotics::geometricNullSpaceControl(const double theta, const Matrix3d& R, const Vector3d& P, const Vector3d& P4, VectorXd& res) const {
    return arm_->geometricNullSpaceControl(theta, R, P, P4, res);
}

Vector6d Robotics::computeToolTor(const VectorXd& joint_pos, const VectorXd& joint_tor){
    Matrix6Xd PHI(6,joint_size_);
    int last_joint_id = joint_size_-1;
    Matrix6Xd J = getJointJacobian(joint_pos, last_joint_id);
    VectorXd gravity = computeGravity(joint_pos, VectorXd::Zero(joint_size_));
    PHI = (J*J.transpose()).inverse() * J;


    VectorXd joint_tor_offset = joint_tor - gravity;
    Vector6d tool_tor = PHI * joint_tor_offset;
    return tool_tor;
}

VectorXd Robotics::ID(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc){
    VectorXd torque = RoboticsBase::ID(joint_pos, joint_vel, joint_acc);
    VectorXd kd = gravity_kd_.cwiseProduct(VectorXd::Zero(joint_size_) - joint_vel);
    return gravity_ratio_.cwiseProduct(torque) + kd;
}

/** Calculate gravity force
 * 
 * \param joint_pos  Joint position
 * \param joint_vel  Joint velocity
 * \param joint_acc  Joint acceleration
 */
VectorXd Robotics::computeGravity(const VectorXd& joint_pos, const VectorXd& joint_vel){
    VectorXd torque = RoboticsBase::ID(joint_pos, joint_vel, VectorXd::Zero(joint_size_));
    // VectorXd tau = torque + friction;
    VectorXd kd_effect = gravity_kd_.cwiseProduct(VectorXd::Zero(joint_size_) - joint_vel);
    VectorXd friction = computeFriction(joint_vel);
    VectorXd tau = gravity_ratio_.cwiseProduct(torque) + kd_effect + friction;

    // for (int i=0; i<joint_size_; i++){
    //     planner_state_.friction[arm_i_][i] =  friction[i];
    // }
    // for (int i=0; i<joint_size_; i++){
    //     planner_state_.gravity_kd_effect[arm_i_][i] =  kd_effect[i];
    // }
    return tau;
}


VectorXd Robotics::computeGravityWithBoundary(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& gravity, const Vector3d& target_tool_pos){
    // SE3 oMf = data_.oMf[payload_frame_idx_]; // Current SE3 transform from world to payload
    // SE3 rot_only = oMf;
    // rot_only.translation() = Vector3d::Zero();

    Vector3d current_pos, force_linear, force_angular;
    Quaterniond current_rot;
    getToolPose(&current_pos, &current_rot);
    double x = gravity_with_boundary_boundary_[0];
    double y = gravity_with_boundary_boundary_[1];
    double z = gravity_with_boundary_boundary_[2];
    Vector3d p = current_pos - target_tool_pos;
    Vector3d err = Vector3d::Zero();
    if (fabs(p[0]) > x){
        err[0] = (p[0] > 0)? p[0] - x : p[0] + x;
    }
    if (fabs(p[1]) > y){
        err[1] = (p[1] > 0)? p[1] - y : p[1] + y;
    }
    if (fabs(p[2]) > z){
        err[2] = (p[2] > 0)? p[2] - z : p[2] + z; 
    }
    force_linear = gravity_with_boundary_kp_.cwiseProduct(err);
    force_angular = Vector3d(0,0,0);
    // Force control_force1; 
    // control_force1.linear() = gravity_with_boundary_kp_.cwiseProduct(err);
    // control_force1.angular() = Vector3d::Zero();
    // Force force1 = control_force1.se3ActionInverse(rot_only);
    // force1 = force1.se3Action(payload_->placement);

    // PINOCCHIO_ALIGNED_STD_VECTOR(Force) body_forces(model_.joints.size(),Force::Zero());
    // body_forces[payload_joint_idx_] = force1;
    // rnea(joint_pos, VectorXd::Zero(joint_size_), VectorXd::Zero(joint_size_), body_forces);
    
    setExternalForce(force_linear, force_angular);
    VectorXd torque = IDWithExternalForces(joint_pos, VectorXd::Zero(joint_size_), VectorXd::Zero(joint_size_));
    VectorXd gravity_kd = gravity_kd_.cwiseProduct(VectorXd::Zero(joint_size_) - joint_vel);
    VectorXd gravity_friction = computeFriction(joint_vel);
    VectorXd gravity_kp = gravity - gravity_kd - gravity_friction;
    VectorXd total_tau =  gravity_ratio_.cwiseProduct(torque);
    VectorXd extern_tau = total_tau - gravity_kp;
    VectorXd sat_extern_tau = saturate(gravity_with_boundary_joint_saturation_, extern_tau);
    VectorXd res = sat_extern_tau + gravity_kp + gravity_friction + gravity_kd;
    // VectorXd gravity_friction = computeFriction(joint_vel);
    // VectorXd gravity_kp = gravity - gravity_kd - gravity_friction;
    // VectorXd total_tau =  gravity_ratio_.cwiseProduct(torque);
    // VectorXd extern_tau = total_tau - gravity_kp;
    // VectorXd jac_tor1 = torque_J*force1.toVector();
    // jac_tor1 *= gravity_ratio_;
    // extern_tau += jac_tor1;
    return res;
}

// /** Calculate impedance force in task space
//  * 
//  * \param joint_pos         Joint position
//  * \param joint_vel         Joint velocity
//  * \param joint_acc         Joint acceleration
//  * \param target_tool_pos   target tool position
//  * \param target_tool_ori   target tool orientation
//  */
// VectorXd Robotics::computeTaskImpedance(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc, const VectorXd& gravity, const Vector3d& target_tool_pos, const Quaterniond& target_tool_ori, float dt){
//     // payload
//     tool_force_.setTarget(target_tool_pos, target_tool_ori);
//     Vector3d current_pos, force_linear, force_angular;
//     Quaterniond current_rot;
//     getToolPose(&current_pos, &current_rot);
//     tool_force_.eval(current_pos, current_rot, dt, &force_linear, &force_angular); // Get Tool force in world frame

//     // Scale the errors by our impedance gain
//     // Force control_force = tool_force_.eval(oMf, dt, planner_state_);
//     // // std::cout << "Tool force in world frame: " << control_force.linear().transpose() << " " << control_force.angular().transpose() << "\n";
    
//     // // Rotate force from world frame to body frame
//     // SE3 rot_only = oMf;
//     // rot_only.translation() = Vector3d::Zero();
//     // Force tool_force_result;
//     // //std::cout << "rotation: \n" << rot_only.rotation() << "\n";
//     // tool_force_result = control_force.se3ActionInverse(rot_only);
//     // // std::cout << "Tool force in tool frame:  " << tool_force_result.linear().transpose() << " " << tool_force_result.angular().transpose() << "\n";
//     // tool_force_result =  tool_force_result.se3Action(payload_->placement);
//     // //std::cout << "Tool force in joint frame: " << tool_force_result.linear().transpose() << " " << tool_force_result.angular().transpose() << "\n";
 

//     // PINOCCHIO_ALIGNED_STD_VECTOR(Force) body_forces(model_.joints.size(),Force::Zero());
//     // body_forces[payload_joint_idx_] = tool_force_result;
//     // rnea(joint_pos, VectorXd::Zero(joint_size_), VectorXd::Zero(joint_size_), body_forces);
//     setExternalForce(force_linear, force_angular);
//     VectorXd torque = IDWithExternalForces(joint_pos, VectorXd::Zero(joint_size_), VectorXd::Zero(joint_size_));

//     VectorXd kd = task_imepdance_joint_additional_kd_.cwiseProduct(VectorXd::Zero(joint_size_) - joint_vel);
//     VectorXd gravity_kd = gravity_kd_.cwiseProduct(VectorXd::Zero(joint_size_) - joint_vel);
//     VectorXd gravity_friction = computeFriction(joint_vel);
//     VectorXd gravity_kp = gravity - gravity_kd - gravity_friction;
//     VectorXd total_tau =  gravity_ratio_.cwiseProduct(torque);
//     VectorXd extern_tau = total_tau - gravity_kp;

//     int last_joint_id = joint_size_-1;
//     Matrix6Xd J = getJointJacobian(joint_pos, last_joint_id);
//     MatrixXd torque_J = J.transpose();
//     // VectorXd jac_tor = torque_J*tool_force_result.toVector();
//     VectorXd jac_tor = VectorXd::Zero(joint_size_);
//     jac_tor =  -gravity_ratio_.cwiseProduct(jac_tor);

//     VectorXd w = VectorXd::Zero(joint_size_);
//     for (int i=0; i < joint_size_; i++){
//         double q_max = joint_limits_(1,i);
//         double q_min = joint_limits_(0,i);
//         double ave_qi = (q_max + q_min)/2;
//         // w[i] = -(1.0/joint_size_)*((joint_pos[i] - ave_qi)/pow((q_max-q_min),2));
//         w[i] = (fabs(joint_pos[i] - ave_qi)/pow((q_max-q_min),2));
//     }
//     VectorXd extern_tau1 = extern_tau.cwiseProduct(w);


//     MatrixXd torque_J1 = torque_J.completeOrthogonalDecomposition().pseudoInverse();
//     MatrixXd torque_J1_inv = torque_J;
//     MatrixXd N = (MatrixXd::Identity(joint_size_, joint_size_) - torque_J1_inv*torque_J1);
    
//     // VectorXd home_q = VectorXd::Zero(joint_size_);
//     VectorXd home_q{0,30,0,60,0,0,0};
//     home_q *= M_PI/180;
//     VectorXd temp = N*(task_null_k_.cwiseProduct(home_q-joint_pos));
//     VectorXd extern_tau2 = extern_tau + temp;
//     // extern_tau = extern_tau2;
    
//     extern_tau = extern_tau.cwiseProduct(task_imepdance_joint_torque_ratio_);
//     VectorXd sat_extern_tau = saturate(task_imepdance_joint_saturation_, extern_tau);
//     VectorXd res = sat_extern_tau + gravity_kp + gravity_friction + kd;
//     return res;
// }

// //???forget what is it
// VectorXd Robotics::computeTaskImpedanceByJacobian(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc, const Vector3d& target_tool_pos, const Quaterniond& target_tool_ori, float dt){
//     // //Haven't verified algorithm on simulation
//     // Vector6d tool_stiffness{1,2,3,4,5,6};
//     // Matrix6Xd J(6,joint_size_);
//     // computeJointJacobians(joint_pos);
//     // getJointJacobian(7, PinocchioReferenceFrame::LOCAL_WORLD_ALIGNED, J);
//     // VectorXd joint_stiffness = J.transpose() * tool_stiffness;

//     // joint_force_.setTarget(target_joint_pos);
//     // joint_force_result_ = joint_force_.eval(joint_pos, joint_vel, dt);
//     // joint_force_result_ = saturate(joint_force_saturation_, joint_force_result_);
//     // VectorXd gravity = computeGravity(joint_pos, VectorXd::Zero(joint_size_));
//     // return joint_force_result_ + gravity;
//     return VectorXd::Zero(0);
// }

/** Calculate impedance force in joint space
 * 
 * \param joint_pos         Joint position
 * \param joint_vel         Joint velocity
 * \param joint_acc         Joint acceleration
 * \param target_joint_pos   target tool position
 */
VectorXd Robotics::computeJointImpedance(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& gravity, const VectorXd& target_joint_pos, float dt){
    joint_force_.setTarget(target_joint_pos);
    joint_force_result_ = joint_force_.eval(joint_pos, joint_vel, dt);
    VectorXd sat_joint_force_result = saturate(joint_force_saturation_, joint_force_result_);
    return sat_joint_force_result + gravity;
}

// VectorXd Robotics::computeTaskAdmittance(const VectorXd& joint_pos, const VectorXd& joint_tor){
//     int last_joint_id = joint_size_-1;
//     Matrix6Xd J = getJointJacobian(joint_pos, last_joint_id);
//     // MatrixXd PHI0 = (J*J.transpose()).inverse() * J;

//     MatrixXd J_inv = getJacobianPseudoInverse(J);
//     MatrixXd J_inv_tr = J_inv.transpose();
//     VectorXd gravity = computeGravity(joint_pos, VectorXd::Zero(joint_size_));
    
//     // VectorXd J_row1 = J_inv_tr.row(1);
//     // if ((joint_pos - VectorXd::Zero(joint_size_)).norm() < 1e-3 || (J_row1 - VectorXd::Zero(joint_size_)).norm() > 60 || (prev_J_inv_tr_row_ - J_row1).norm() > 1e-3){
//     //     j_offset = VectorXd::Zero(joint_size_);
//     //     t_offset = Vector6d::Zero();
//     //     t_vel = Vector6d::Zero();
//     //     j_vel = VectorXd::Zero(joint_size_);
//     //     j_inv_trans = VectorXd::Zero(joint_size_);
//     //     prev_J_inv_tr_row_ = J_row1;
//     //     return VectorXd::Zero(joint_size_);
//     // }
    
//     VectorXd joint_tor_offset = joint_tor - gravity;
//     Vector6d tool_tor_offset = J_inv_tr * joint_tor_offset;
//     Vector6d tool_vel = task_admittance_kp_ * tool_tor_offset;
//     VectorXd joint_vel_cmd = J_inv * tool_vel;

//     std::cout << "J: \n";
//     for (int i =0 ; i < 6; i++){
//         std::cout << J.row(i)<< std::endl;
//     }
//     std::cout << "\nPHI: \n";
//     for (int i =0 ; i < 6; i++){
//         std::cout << J_inv_tr.row(i) << std::endl;
//     }

//     prev_J_inv_tr_row_ = J_inv_tr.row(1);
//     return joint_vel_cmd;
// }

// VectorXd Robotics::computeJointAdmittance(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_tor, const VectorXd& gravity, const VectorXd& last_joint_vel_cmd,  float dt){
//     // MatrixXd M = crba(model_, data_, joint_pos);
//     // // VectorXd gravity = computeGravity(joint_pos, joint_vel);
//     // VectorXd gravity = computeGravity(joint_pos, VectorXd::Zero(joint_size_));
//     // VectorXd diff = joint_tor - gravity;
//     // VectorXd a = getJacobianPseudoInverse(M)*diff;
//     // float dt = 0.005;
//     // VectorXd joint_vel_cmd = joint_vel + a*dt;

//     // for (int i=0; i < joint_vel_cmd.size(); i++){
//     //     float sgn = (joint_vel_cmd[i] < 0)? -1: 1;
//     //     joint_vel_cmd[i] = sgn * std::min(fabs(joint_vel_cmd[i]), joint_admittance_max_speed_[i]);
//     // }

//     // VectorXd joint_vel_cmd = VectorXd::Zero(joint_size_);
//     // VectorXd gravity = computeGravity(joint_pos, VectorXd::Zero(joint_size_));
//     // double err1, err2, sgn, adjusted_err = 0;
//     // for (int i=0; i < joint_tor.size(); i++){
//     //     err1 = joint_tor[i] - gravity[i];
//     //     err2 = fabs(err1) - joint_admittance_threshold_[i];
//     //     if (err2 > 0){
//     //         sgn = (err1 < 0)? -1.0 : 1.0;
//     //         sgn *= (i == 0 || i == 5|| i == 6)? 1.0 : -1; //Joint1,2,7 is inverted sign
//     //         adjusted_err = std::min(fabs(err1), (double)joint_admittance_force_upper_bound_[i]);
//     //         adjusted_err *= M_PI/joint_admittance_force_upper_bound_[i];
//     //         joint_vel_cmd[i] = joint_admittance_max_speed_[i]*sgn*(1-std::cos(adjusted_err));
//     //     }
//     // }

//     // return joint_vel_cmd;

//     // VectorXd joint_vel_cmd = VectorXd::Zero(joint_size_);
//     // VectorXd gravity_kd = gravity_kd_.cwiseProduct(VectorXd::Zero(joint_size_) - joint_vel);
//     // VectorXd gravity_friction = computeFriction(joint_vel);
//     // VectorXd gravity_kp = gravity - gravity_kd - gravity_friction;
//     // double err, sgn0, sgn, sgn1, adjusted_err, damping = 0;
//     // for (int i=0; i < joint_tor.size(); i++){
//     //     sgn0 = motor_direction_[i]; 
//     //     damping = joint_admittance_kd_ * (0 - joint_vel[i]*180/M_PI); //Damping force  
//     //     err = joint_tor[i] - gravity_kp[i];
//     //     if ((fabs(err) - joint_admittance_threshold_[i]) > 0){
//     //         adjusted_err =  err*joint_admittance_extern_force_k_[i];
//     //         adjusted_err += damping*sgn0; //Sig0 for switching between velocity to torque
//     //         sgn = (adjusted_err < 0)? -1*sgn0: 1*sgn0;
//     //         adjusted_err = std::min(fabs(adjusted_err), (double)joint_admittance_force_upper_bound_[i]);
//     //         adjusted_err *= M_PI/joint_admittance_force_upper_bound_[i];
//     //         joint_vel_cmd[i] = last_joint_vel_cmd[i] + joint_admittance_max_acceleration_[i]*sgn*(1-std::cos(adjusted_err))*dt;

//     //         sgn1 = (joint_vel_cmd[i] < 0)? -1.0 : 1.0;
//     //         joint_vel_cmd[i] = sgn1 * std::min(fabs(joint_vel_cmd[i]), joint_admittance_max_speed_[i]);
//     //     }
//     // }


//     // return joint_vel_cmd;
// }

// VectorXd Robotics::computeNullAdmittance(const VectorXd& joint_pos, const VectorXd& joint_tor, const Matrix3d& R, const Vector3d& P, const Vector3d& P4, double& last_total_err, float dt){
//     VectorXd null_joint_pos = joint_pos;
//     VectorXd gravity = computeGravity(joint_pos, VectorXd::Zero(joint_size_));
//     int ji = null_admittance_kp_ji_;
//     float err = joint_tor[ji] - gravity[ji];
//     double sgn0 = (ji == 0 || ji == 5|| ji == 6)? 1.0 : -1; //Torque and velocity of J3 is in opposite direction.
//     // float k = 0.05*dt;

//     std::cout << "err: "<< err << std::endl;
//     std::cout << "larget than threshold: "<< (fabs(err) - null_admittance_threshold_ > 0) << std::endl;
//     if (fabs(err) - null_admittance_threshold_ > 0){
//         err = null_admittance_kp_ * err * dt;
//         last_total_err += RoboticsBase::Sgn(err) * sgn0 * std::min(fabs(err), null_admittance_saturation_*dt);
//         std::cout << "err1: "<< RoboticsBase::Sgn(err) * sgn0 * std::min(fabs(err), null_admittance_saturation_) << std::endl;
//         // last_total_err += Sgn(err) * sgn0 * k;
//         // last_total_err += sgn0 * k;
//         arm_->geometricNullSpaceControl(last_total_err, R, P, P4, null_joint_pos);
//     }
//     printf("last_total_err: %lf \n", last_total_err);
//     std::cout << "null_joint_pos: " << null_joint_pos.transpose()<<"\n\n";
//     return null_joint_pos;
// }

VectorXd Robotics::computeFriction(const VectorXd& joint_vel){
    //First order low pass filter
    filtered_joint_vel_ = vel_low_pass_filter_ratio_ * filtered_joint_vel_ + (1 - vel_low_pass_filter_ratio_) * joint_vel.array();
    //Deadband filter
    ArrayXd temp = filtered_joint_vel_;
    filtered_joint_vel_ = (filtered_joint_vel_.abs()<0.002).select(0.0, filtered_joint_vel_);
    // for (int i=0; i<joint_size_; i++){
    //     planner_state_.filtered_joint_vel[arm_i_][i] =  filtered_joint_vel_[i];
    // }
    
    ArrayXd signs = (filtered_joint_vel_ == 0).select(0.0, filtered_joint_vel_.sign());
    switch (friction_model_){
        case FrictionModel::kCoulumbWithViscous:
            return friction_cv_ratio_ * (friction_cv_coulumb_*signs + friction_cv_viscous_*filtered_joint_vel_);
            break;
        default:
            break;
    }
    return VectorXd::Zero(joint_size_);
}


VectorXd Robotics::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const{
    return arm_->convertMotorJoint(X, convert_to, is_tau);
}