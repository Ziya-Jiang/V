//Pinocchio/Eigen headers MUST be at the absolute top for PCH to trigger
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/model.hpp"

#include "pinocchio_wrapper.h"
#include <iostream>

struct PinocchioWrapper::Impl {
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::Frame* tool_frame_;             // The frame that holds the payload
    pinocchio::JointIndex tool_joint_idx_;   
    pinocchio::FrameIndex tool_frame_idx_;   
    PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) body_forces_;
    pinocchio::FrameIndex ee_frame_idx_;
    
    Impl(const std::string& urdf_file, const std::vector<std::string>& joint_names, const std::string& ee_link_name, const Vector3d& tool_offset){
        pinocchio::Model urdf_model;
        pinocchio::urdf::buildModel(urdf_file, urdf_model);
        
        //Build a reduced model
        std::vector<pinocchio::JointIndex> joints_to_lock_ids;
        for (pinocchio::JointIndex i = 1; i < (pinocchio::JointIndex)urdf_model.njoints; ++i) {
            std::string name = urdf_model.names[i];
            auto it = std::find(joint_names.begin(), joint_names.end(), name);
            if (it == joint_names.end()) {
                joints_to_lock_ids.push_back(i);
            }
        }
        Eigen::VectorXd q_ref = pinocchio::neutral(urdf_model); // Position for the locked arm
        pinocchio::buildReducedModel(urdf_model, joints_to_lock_ids, q_ref, model_);
        model_.gravity.linear(Vector3d(0,0,-9.80665));
        ee_frame_idx_ = model_.getFrameId(ee_link_name);
        addToolFrame(ee_link_name, tool_offset);

        data_ = pinocchio::Data(model_); 
        int joint_size = model_.nq;
        centerOfMass(model_, data_, true);
    }

    void addToolFrame(const std::string& ee_link_name, const Vector3d& tool_offset){
        // Add payload frame to end effector
        pinocchio::Frame& ee_frame = model_.frames[ee_frame_idx_];
        pinocchio::JointIndex ee_joint_idx = model_.frames[ee_frame_idx_].parentJoint;

        pinocchio::SE3 tool_frame_from_ee = pinocchio::SE3::Identity();
        tool_frame_from_ee.translation() = tool_offset;
        // tool_frame_from_ee.rotation() = Eigen::Quaterniond(1,0,0,0);

        // Pinocchio frames are stored relative to their parent JOINT.
        pinocchio::SE3 tool_frame_from_ee_joint = ee_frame.placement * tool_frame_from_ee;

        tool_frame_idx_ = model_.addFrame(pinocchio::Frame("tool", ee_joint_idx, ee_frame_idx_, tool_frame_from_ee_joint, pinocchio::FrameType::OP_FRAME, pinocchio::Inertia::Zero()));
        tool_joint_idx_ = ee_joint_idx;
        tool_frame_ = &(model_.frames[tool_frame_idx_]);
        body_forces_ = PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force)(model_.joints.size(), pinocchio::Force::Zero());
    }

    Vector6d calPoseDifference_internal(const pinocchio::SE3 T1, const pinocchio::SE3 T2){
        pinocchio::SE3 dT = T1.actInv(T2);
        return log6(dT).toVector();
    }

    pinocchio::SE3 FK_internal(const VectorXd& joint_pos, int joint_id){
        pinocchio::framesForwardKinematics(model_, data_, joint_pos);
        return data_.oMi[joint_id+1];
    }
};

PinocchioWrapper::PinocchioWrapper(const std::string& urdf_file, const std::vector<std::string>& joint_names,
    const std::string& ee_link_name, const Vector3d& tool_offset){
    pimpl_ = std::make_unique<Impl>(urdf_file, joint_names, ee_link_name, tool_offset);

    joint_size_ = pimpl_->model_.nq;
    FK(VectorXd::Zero(joint_size_));
    
    //Assign base class parameters
    link_lengths_ = Eigen::ArrayXf::Zero(joint_size_);
    joint_revolute_ = Eigen::Array<bool, Eigen::Dynamic, 1>::Zero(joint_size_);
    for (int i=1; i<pimpl_->model_.njoints; i++){ //joint 0 is the universe
        std::string t = pimpl_->model_.joints[i].shortname();
        t.pop_back();
        joint_revolute_[i-1] = (t == "JointModelR");
        link_lengths_[i-1] = static_cast<float>(pimpl_->data_.oMi[i].translation().transpose()[2] - pimpl_->data_.oMi[i-1].translation().transpose()[2]);
    }

    //Get joint lower and upper limits  
    joint_lower_limit_.resize(joint_size_);
    joint_upper_limit_.resize(joint_size_);
    for (pinocchio::JointIndex joint_id = 1; joint_id < joint_size_; ++joint_id) {
        joint_lower_limit_[joint_id-1] = pimpl_->model_.lowerPositionLimit[joint_id];
        joint_upper_limit_[joint_id-1] = pimpl_->model_.upperPositionLimit[joint_id];
    }
}

//Input: joint position in radius
MatrixX7d PinocchioWrapper::FK(const VectorXd& joint_pos, Vector7d* ee_pose, Vector7d* tool_pose){
    pinocchio::framesForwardKinematics(pimpl_->model_, pimpl_->data_, joint_pos);
    MatrixX7d pose(pimpl_->model_.nv, 7);
    for(pinocchio::JointIndex joint_id = 1; joint_id < (pinocchio::JointIndex)pimpl_->model_.njoints; ++joint_id){
        int i = (int)joint_id - 1;
        Vector3d pos = pimpl_->data_.oMi[joint_id].translation();
        Eigen::Quaterniond ori(pimpl_->data_.oMi[joint_id].rotation());
        pose.row(i) << pos.transpose(), ori.w(), ori.x(), ori.y(), ori.z();
    }

    if (ee_pose != nullptr){
        Vector3d pos = pimpl_->data_.oMf[pimpl_->ee_frame_idx_].translation();
        Eigen::Quaterniond ori(pimpl_->data_.oMf[pimpl_->ee_frame_idx_].rotation());
        *ee_pose << pos, ori.w(), ori.x(), ori.y(), ori.z();
    }

    if (tool_pose != nullptr){
        Vector3d pos = pimpl_->data_.oMf[pimpl_->tool_frame_idx_].translation();
        Eigen::Quaterniond ori(pimpl_->data_.oMf[pimpl_->tool_frame_idx_].rotation());
        *tool_pose << pos, ori.w(), ori.x(), ori.y(), ori.z();
    }
    return pose;
}

Matrix4d PinocchioWrapper::FK(const VectorXd& joint_pos, int joint_id){
    pinocchio::framesForwardKinematics(pimpl_->model_, pimpl_->data_, joint_pos);
    return pimpl_->data_.oMi[joint_id+1].toHomogeneousMatrix();
}

Matrix6Xd PinocchioWrapper::getJointJacobian(const VectorXd& joint_pos, int joint_id){
    Matrix6Xd J = Matrix6Xd::Zero(6, pimpl_->model_.nv);
    pinocchio::computeJointJacobians(pimpl_->model_, pimpl_->data_, joint_pos);
    pinocchio::getJointJacobian(pimpl_->model_, pimpl_->data_, joint_id+1, pinocchio::ReferenceFrame::LOCAL, J);
    return J;
}

//Compute inverse dynamics using Recursive Newton-Euler algorithm.
VectorXd PinocchioWrapper::ID(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc){
    pinocchio::rnea(pimpl_->model_, pimpl_->data_, joint_pos, joint_vel, joint_acc);
    return pimpl_->data_.tau;
}

VectorXd PinocchioWrapper::IDWithExternalForces(const VectorXd& joint_pos,const VectorXd&  joint_vel,const VectorXd& joint_acc){
    pinocchio::rnea(pimpl_->model_, pimpl_->data_, joint_pos, joint_vel, joint_acc, pimpl_->body_forces_);
    return pimpl_->data_.tau;
}

/** Calculate inverse kinematics with Newton–Raphson (NR) root-finding algorithm (i.e. second-order Jacobian-based).
 * 
 * \param R     target 3x3 orientation matrix from base to ee frame.
 * \param P     target 3x1 translation vector from base to ee frame.
 * \param res   input current joint position, return the calculated joint postion.
 * \returns     state of IK
 */
bool PinocchioWrapper::IK(const Matrix3d& R, const Vector3d& P, VectorXd& res){
    assert(res.size() == pimpl_->model_.nq && "Input configuration size mismatch!");
    // Get the R,P in the frame of the last joint
    const pinocchio::SE3 oMdes_frame(R, P);
    const pinocchio::SE3& joint_to_frame = pimpl_->model_.frames[pimpl_->ee_frame_idx_].placement;
    const pinocchio::SE3 oMdes = oMdes_frame * joint_to_frame.inverse();
    
    const pinocchio::JointIndex last_joint_id = joint_size_ - 1;
    const double eps  = 2e-4;
    const int IT_MAX  = 200;
    const double DT   = 3e-1;
    const double damp = 1e-6;
    
    // Safety thresholds for distant targets
    const double MAX_ERROR_STEP = 1.0; // Caps maximum linear/angular step per iteration

    int i = 0;
    Vector6d err;
    VectorXd q = res;
    bool success = false;
    VectorXd v(pimpl_->model_.nv);
    Matrix6Xd J(6, pimpl_->model_.nv);
    Matrix66 JJt;
    pinocchio::SE3 T;
    J.setZero();

    while (i < IT_MAX ){
        T = pimpl_->FK_internal(q, static_cast<int>(last_joint_id));
        err = pimpl_->calPoseDifference_internal(oMdes, T);
        
        // Check for convergence
        if(err.norm() < eps){
            success = true;
            break;
        }

        // Clamp error magnitude to prevent violent velocity spikes for far targets
        if(err.norm() > MAX_ERROR_STEP) {
            err *= (MAX_ERROR_STEP / err.norm());
        }

        // Compute Damped Least Squares update
        J = getJointJacobian(q, static_cast<int>(last_joint_id));
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
        
        // // 1. Primary IK Task
        // Eigen::VectorXd v_task = J_pinv * err;

        // // 2. Compute the Null-Space Projection Matrix
        // Eigen::MatrixXd I = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
        // Eigen::MatrixXd Null_Space = I - J_pinv * J;

        // // 3. Secondary Task: Push joints toward the center of their limits
        // Eigen::VectorXd v_limit_avoidance = Eigen::VectorXd::Zero(model_.nv);
        // double gain = 10.0; // Tuning parameter

        // for(int i=0; i < model_.nv; ++i) {
        //     double q_center = (model_.upperPositionLimit[i] + model_.lowerPositionLimit[i]) / 2.0;
        //     // Velocity points toward the center of the joint's range
        //     v_limit_avoidance[i] = gain * (q_center - q[i]); 
        // }

        // // 4. Combine: The primary task takes priority, secondary task uses remaining DOF
        // Eigen::VectorXd v_final = v_task + Null_Space * v_limit_avoidance;

        // Safely integrate and strictly enforce robot joint limits
        VectorXd q_next = pinocchio::integrate(pimpl_->model_, q, v * DT);
        q_next = q_next.cwiseMax(pimpl_->model_.lowerPositionLimit).cwiseMin(pimpl_->model_.upperPositionLimit);
        q = q_next;
        i++;
    }
    
    if (success) res = q;
    return success;
}

Vector6d PinocchioWrapper::calPoseDifference(const Matrix3d& R1, const Vector3d& P1, const Matrix3d& R2, const Vector3d& P2){
    pinocchio::SE3 oMdes1(R1,P1);
    pinocchio::SE3 oMdes2(R2,P2);
    return pimpl_->calPoseDifference_internal(oMdes1, oMdes2);
}

void PinocchioWrapper::getToolPose(Vector3d* pos, Eigen::Quaterniond* rot){
    pinocchio::SE3 oMf = pimpl_->data_.oMf[pimpl_->tool_frame_idx_];              // Current SE3 transform from world to payload
    *pos = oMf.translation();
    *rot = oMf.rotation();
}

void PinocchioWrapper::updateToolFrame(const Vector3d& tool_offset){
    pinocchio::Frame& ee_frame = pimpl_->model_.frames[pimpl_->ee_frame_idx_];
    pinocchio::SE3 tool_frame_from_ee(Eigen::Matrix3d::Identity(), tool_offset);
    pinocchio::SE3 tool_frame_from_ee_joint = ee_frame.placement * tool_frame_from_ee;
    pimpl_->tool_frame_->placement = tool_frame_from_ee_joint;
}

VectorXd PinocchioWrapper::getJointLowerLimit(){
    return joint_lower_limit_;
}

VectorXd PinocchioWrapper::getJointUpperLimit(){
    return joint_upper_limit_;
}

void PinocchioWrapper::setExternalForce(const Vector3d& force_linear, const Vector3d& force_angular){
    // Rotate force from world frame to body frame
    pinocchio::Force world_frame_force;
    world_frame_force.linear() = force_linear;
    world_frame_force.angular() = force_angular;
    pinocchio::SE3 rot_only = pimpl_->data_.oMf[pimpl_->tool_frame_idx_]; // Current SE3 transform from world to payload
    rot_only.translation() = Vector3d::Zero();
    pinocchio::Force tool_frame_force = world_frame_force.se3ActionInverse(rot_only); // Tool force in tool frame
    pinocchio::Force joint_frame_force =  tool_frame_force.se3Action(pimpl_->tool_frame_->placement); // Tool force in joint frame
    pimpl_->body_forces_[pimpl_->tool_joint_idx_] = joint_frame_force;
}