#ifndef ROBOTICS_H
#define ROBOTICS_H

#include <iostream>

#include "robotics/robot/arm.h"
#include "robotics/robotics_base.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"

namespace robo{
    enum FrictionModel{kCoulumbWithViscous};
    using namespace Eigen;

    /** Simple PD controller for impedance force */
    class ImpedanceForcePdControl{
        public:
            Vector3d pos_error_ {0,0,0};
            Vector3d rot_error_ {0,0,0};
            Vector3d prev_pos_error_ {0,0,0};
            Vector3d prev_rot_error_ {0,0,0};
            Vector3d pos_error_d_ {0,0,0};
            Vector3d rot_error_d_ {0,0,0};
            Vector3d pos_error_i_ {0,0,0};
            Vector3d rot_error_i_ {0,0,0};
            float ki_ratio_ = 1;
            Vector6d kp_ {200,200,200,5,5,5};
            Vector6d kd_ {0,0,0,0,0,0};
            Vector6d ki_ {0,0,0,0,0,0};
            Vector3d target_pos_;
            Quaterniond target_rot_;

            void setTarget(const Vector3d& pos, const Quaterniond& rot){
                target_pos_ = pos;
                target_rot_ = rot;
            }
            
            /** Evaluate the PD impedance controller
             * 
             * \param xform Current SE3 Transform of frame
             * \param dt    Time period from last eval to this eval
             * \return      Control force to move towards target
             */
            void eval(const Vector3d& pos, const Quaterniond& rot, double dt, Vector3d* force_linear, Vector3d* force_angular){
                // Calculate the translational and rotational error => direction and magnitude of force/moment before scaling by gain
                pos_error_ = target_pos_ - pos;
                rot_error_ = target_rot_.greatcircleAxis(rot).transpose() * target_rot_.greatcircleDistance(rot);
                pos_error_d_ = (pos_error_ - prev_pos_error_)/dt;
                rot_error_d_ = (rot_error_ - prev_rot_error_)/dt;
                prev_pos_error_ = pos_error_;
                prev_rot_error_ = rot_error_;

                // pos_error_i_ = (1-ki_ratio_)*pos_error_i_ + ki_ratio_*pos_error_;
                // rot_error_i_ = (1-ki_ratio_)*rot_error_i_ + ki_ratio_*rot_error_;

                // Scale the errors by impedance gain
                *force_linear = kp_.head(3).cwiseProduct(pos_error_)*-1.0 + kd_.head(3).cwiseProduct(pos_error_d_)*-1.0;
                *force_angular = kp_.tail(3).cwiseProduct(rot_error_)*-1.0 + kd_.tail(3).cwiseProduct(rot_error_d_)*-1.0;
                
                // control_force_.linear() = kp_.head(3).cwiseProduct(pos_error_)*-1.0 + kd_.head(3).cwiseProduct(pos_error_d_)*-1.0 + ki_.head(3).cwiseProduct(pos_error_i_)*-1.0;
                // control_force_.angular() = kp_.tail(3).cwiseProduct(rot_error_)*-1.0 + kd_.tail(3).cwiseProduct(rot_error_d_)*-1.0 + ki_.tail(3).cwiseProduct(rot_error_i_)*-1.0;
                
            }

    };

    class JointImpedanceForcePdControl{
        public:
            VectorXd pos_error_ = VectorXd::Zero(7);
            VectorXd kp_ = VectorXd::Zero(7);
            VectorXd kd_ = VectorXd::Zero(7);
            VectorXd ki_ = VectorXd::Zero(7);
            VectorXd target_;
            void setTarget(const VectorXd& target_pos){
                target_ = target_pos;
            }
            
            /** Evaluate the PD impedance controller
             * 
             * \param joint_pos Current robot joint position
             * \param dt        Time period from last eval to this eval
             * \return          Control force to move towards target
             */
            VectorXd eval(VectorXd joint_pos, VectorXd joint_vel, double dt){
                pos_error_ = target_ - joint_pos;
                VectorXd control_force(joint_pos.size());
                for (int i=0; i<joint_pos.size(); i++){
                    control_force[i] = kp_[i]*pos_error_[i] - kd_[i]*joint_vel[i];
                }
                return control_force;
            }
    };

    class Robotics: public RoboticsBase{
        public:
            Robotics(const std::string& urdf_file, const std::vector<std::string>& joint_names, const std::string& ee_link_name,
                const Vector3d& tool_offset, int arm_i, int joint_size, const Matrix2Xf& joint_limits, const std::string& library_name);
            ~Robotics();
            //Setters
            void setArm(const std::string& name, const VectorXf gear_ratio);
            void setGravityGains(const VectorXd& ratio, const VectorXd& kd);
            void setTaskImepdance(const Vector6d& tool_kp, const Vector6d& tool_kd, const Vector6d& tool_ki, float ki_ratio, const VectorXd& joint_saturation, const VectorXd& joint_torque_ratio, const VectorXd& joint_additional_kd, const VectorXd& task_null_k);
            void setJointImpedance(const VectorXd& joint_kp, const VectorXd& joint_kd, const VectorXd& joint_ki);
            void setGravityWithBoundary(const Vector3d& kp, const Vector3d& boundary, const VectorXd& joint_saturation);
            void setTaskAdmittance(float kp);
            void setJointAdmittance(float damping, const VectorXd& upper_bound, const VectorXd& threshold, const VectorXd& max_speed, const VectorXd& max_acceleration, const VectorXd& extern_for_k);
            void setNullAdmittance(float kp, float kd, float threshold, float saturation, int joint_index);
            void setJointSaturation(const VectorXd& joint_saturation);
            void setJointVelLowPassFilterRatio(const VectorXd& ratio);
            void setFrictionAsCoulumbWithViscous(const VectorXd& ratio, const VectorXd& fc, const VectorXd& fv);
            
            //Static functions
            static MatrixX6d getJacobianPseudoInverse(MatrixXd J);

            VectorXd ID(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc);
            VectorXd jacobianNullSpaceControl(const VectorXd& joint_pos, const VectorXd& joint_vel_tar);
            bool geometricNullSpaceControl(const double theta, const Matrix3d& R, const Vector3d& P, const Vector3d& P4, VectorXd& res) const;
            Vector6d computeToolTor(const VectorXd& joint_pos, const VectorXd& joint_tor);
            VectorXd computeGravity(const VectorXd& joint_pos, const VectorXd& joint_vel);
            VectorXd computeGravityWithBoundary(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& gravity, const Vector3d& target_tool_pos);
            // VectorXd computeTaskImpedance(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc, const VectorXd& gravity, const Vector3d& target_tool_pos, const Quaterniond& target_tool_ori, float dt);
            // VectorXd computeTaskImpedanceByJacobian(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc, const Vector3d& target_tool_pos, const Quaterniond& target_tool_ori, float dt);
            VectorXd computeJointImpedance(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& gravity, const VectorXd& target_joint_pos, float dt);
            // VectorXd computeTaskAdmittance(const VectorXd& joint_pos, const VectorXd& joint_tor);
            // VectorXd computeJointAdmittance(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_tor, const VectorXd& gravity, const VectorXd& last_joint_vel_cmd, float dt);
            // VectorXd computeNullAdmittance(const VectorXd& joint_pos, const VectorXd& joint_tor, const Matrix3d& R, const Vector3d& P, const Vector3d& P4, double& last_total_err, float dt);
            VectorXd computeFriction(const VectorXd& joint_vel);
            VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const;

        protected:
            Arm* arm_ = nullptr;
            int arm_i_;
            int joint_size_;
            Matrix2Xf joint_limits_; 
            
            //gravity
            VectorXd gravity_ratio_;
            VectorXd gravity_kd_;

            //gravity with boundary
            Vector3d gravity_with_boundary_kp_;
            Vector3d gravity_with_boundary_boundary_;
            VectorXd gravity_with_boundary_joint_saturation_;

            //Task admittance 
            VectorXd prev_J_inv_tr_row_;
            float task_admittance_kp_;

            //Joint admittance
            float joint_admittance_kd_;
            VectorXd joint_admittance_force_upper_bound_;
            VectorXd joint_admittance_threshold_;
            VectorXd joint_admittance_max_speed_;
            VectorXd joint_admittance_max_acceleration_;
            VectorXd joint_admittance_extern_force_k_;

            //Null admittance
            float null_admittance_kp_;
            float null_admittance_kd_;
            float null_admittance_threshold_;
            float null_admittance_saturation_;
            int null_admittance_kp_ji_;

            // //Task Impedance
            // Inertia no_payload_;        // A record of what the frame inertia was before we added our payload. 
            // Frame* payload_;            // The frame that holds the payload
            // JointIndex payload_joint_idx_;   // Index of the joint with the payload
            // FrameIndex payload_frame_idx_;   // Index of the frame with the payload
            ImpedanceForcePdControl tool_force_; // Payload force controller
            Vector6d task_imepdance_kp_;
            Vector6d task_imepdance_kd_;
            Vector6d task_imepdance_ki_;
            float task_imepdance_ki_ratio_;
            VectorXd task_imepdance_joint_saturation_;
            VectorXd task_imepdance_joint_torque_ratio_;
            VectorXd task_imepdance_joint_additional_kd_;
            VectorXd task_null_k_;

            //Joint impedance
            JointImpedanceForcePdControl joint_force_; // Payload force controller
            VectorXd joint_force_result_;             // Copy of force result for monitoring
            VectorXd joint_force_saturation_;         // Force values to clamp at

            //joint vel low pass filter
            ArrayXd vel_low_pass_filter_ratio_;
            ArrayXd filtered_joint_vel_;

            //friction
            FrictionModel friction_model_;
            ArrayXd friction_cv_ratio_;
            ArrayXd friction_cv_coulumb_;
            ArrayXd friction_cv_viscous_;

            float k_, alpha_, omega1_, omega2_;
    };
}
using robo::Robotics;

#endif