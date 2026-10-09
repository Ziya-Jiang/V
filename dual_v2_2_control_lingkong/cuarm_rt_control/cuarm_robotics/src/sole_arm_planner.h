#ifndef SOLE_ARM_PLANNER_H
#define SOLE_ARM_PLANNER_H
#include <iostream>
#include <vector>
#include <array>

#include <yaml-cpp/yaml.h>

#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "interpolation.h"
#include "robotics/robotics.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "arm_planner.h"

class SoleArmPlanner: public ArmPlanner{
    public:
        SoleArmPlanner(PlannerCommand& pl_cmd, PlannerState& pl_state, float dt, const YAML::Node& config, const std::string& config_prefix_path, const std::string& urdf_path, const std::string& library_name, int arm_i);
        ~SoleArmPlanner();
        void update(const PanelCommand& pa_cmd, const RobotState& ro_state) override;
        VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const;
        VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau, int arm_i) const override;
        void updatePlannerState(const RobotState& ro_state);
        MotionControl getLastMotionType();
    protected:
        void computeFilteredTorque();
        void setJointVal(const VectorXd& tar);
        VectorXd getPIDCommand(const VectorXd& tar);
        void setTaskLineVal(const Vector6d& tar, LineMoveStrategy strategy);
        void setTaskVal(const Vector6d& tar);
        bool setTaskValInternal(const Vector6d& tar, VectorXd& q);
        void setNullVal(const VectorXd& tar);
        void setPlaybackVal();
        void setMotorVal(const VectorXd& tar);
        void resetControlMem(MotionControl t);
        void setGravityWithBoundaryTarget();
        void setImpedanceTarget();

        VectorXd getSlicedMotorVal(VectorXd target, VectorXd x0, VectorXd v0, VectorXd tor0);
        VectorXd guardJointPositionLimits(VectorXd val);
        VectorXd PIDControl(VectorXd val, VectorXd x0, VectorXd v0, VectorXd tor0);
        void updateRobotCmdTarget(VectorXd val);
        void updateMotorInSimulation(VectorXd val);

        void computeFK(const VectorXd& joint_pos);
        void computeFK(const VectorXd& joint_pos, Vector7d& ee_pose, Vector7d& tool_pose) const;
        bool computeQuIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const;
        void computeID();
        bool reachJointLimit(const VectorXd& joint_pos, VectorXi& reached_limit_id);
        VectorXd saturate(const VectorXd& v, bool is_top);
        VectorXd limitJointJump(const VectorXd& v, bool is_top, float k=1.0);
        VectorXd clampJointPos(const VectorXd&v);

        float dt_;
        bool is_simulation_ = true;
        Robotics* robotics_;
        PlannerState& planner_state_;
        PlannerCommand& planner_command_;
        uint32_t sequence_id_ = 0;

        int arm_i_;
        int joint_size_;
        Matrix2Xf joint_pos_limits_;
        Matrix2Xf joint_vel_limits_;
        Matrix2Xf joint_acc_limits_;
        Matrix2Xf joint_tor_limits_;
        VectorXf joint_jump_pos_limits_;
        VectorXf joint_jump_vel_limits_;
        VectorXf joint_jump_tor_limits_;
        Vector3f task_motion_limit_linear_velocity_;
        Vector3f task_motion_limit_angular_velocity_;

        MotionControl motion_type_      = MotionControl::kJoint;
        OrientControl task_orient_type_ = OrientControl::kEnd;
        ControlType target_type_        = ControlType::kPosition;
        ControlType actuator_mode_       = ControlType::kPosition;
        ControlType joint_jump_last_actuator_mode_   = ControlType::kInvalid;

        VectorXd pos_vel_kp_;
        VectorXd pos_tor_kp_;
        VectorXd vel_tor_kp_;
        
        VectorXd pos_vel_kd_;
        VectorXd pos_tor_kd_;
        VectorXd vel_tor_kd_;

        VectorXd sliced_joint_tar_;
        VectorXd sliced_null_tar_;
        VectorXd sliced_motor_tar_;
        Vector6d sliced_task_tar_;
        Matrix3d task_base_R_;
        Vector3d task_base_rpy_;
        
        VectorXd motor_tar_;
        VectorXd joint_cmd_;
        Vector6d task_cmd_;
        VectorXd motor_cmd_;
        VectorXd pid_cmd_;
        VectorXd motor_cmd_final_;
        VectorXd joint_jump_last_joint_cmd_;

        VectorXd filtered_motor_tor_; 
        VectorXd filtered_joint_tor_;    
        VectorXd filtered_motor_tor_kalman_;
        VectorXd filtered_joint_tor_kalman_;
        float kalman_p_ = 0, kalman_g_ = 0;

        VectorXd filtered_motor_tor_euro_;
        VectorXd filtered_joint_tor_euro_;
        VectorXd euro_x_prev_, euro_dx_prev_;

        VectorXd joint_tor_gravity_;     // joint tau due to gravity at current pose
        VectorXd motor_tor_gravity_;
        VectorXd motor_tor_offset_;      // motor tau at current pose
        VectorXd motor_vel_offset_;      // motor vel for admittance control
        VectorXd motor_pos_;         // motor position
        VectorXd motor_vel_;         // motor velocity
        VectorXd motor_acc_;         // motor acceleration
        VectorXd motor_tor_;         // motor torque
        VectorXd joint_pos_;         // joint position
        VectorXd joint_vel_;         // joint velocity
        VectorXd joint_acc_;         // joint acceleration
        VectorXd joint_tor_;         // joint torque
        VectorXd last_joint_pos_;    // last joint position
        VectorXd last_joint_vel_;    // last joint veclocity
        VectorXd last_motor_vel_;    
        MatrixX7d joint_pose_;
        Vector7d ee_pose_;
        Vector7d tool_pose_;
        VectorXd task_base_joint_pos_;      // saved joint_pos when swtich to Task Control
        VectorXd task_base_end_pose_;       // saved ToolPose when swtich to Task Control
        std::vector<VectorXd> last_task_waypoint_cmd_;
        std::vector<VectorXd> last_task_waypoint_res_;


        bool is_reached_limits_ = false;
        VectorXd limited_joint_vel_;
        VectorXd limited_joint_tor_;
        VectorXi reached_limit_ji_;
        VectorXd last_sat_joint_cmd_;
        VectorXd last_task_solved_joint_cmd_;

        Vector3d null_base_P_;   // saved P07 when swtich to Null Control
        Matrix3d null_base_R_;   // saved R07 when swtich to Null Control
        Vector3d null_base_P4_;  // saved P04 when swtich to Null Control

        Vector3d gravity_with_boundary_target_pos_;
        Vector3d task_impedance_target_pos_;
        Eigen::Quaterniond task_impedance_target_ori_;
        VectorXd joint_impedance_target_;
        VectorXd admittance_last_cmd_;
        
        ControlAlgorithm control_algorithm_ = ControlAlgorithm::kNone;

        float admittance_force_upper_bound_ = 0;
        float admittance_threshold_ = 0;
        float admittance_max_speed_ = 0;
        
        float acc_t_ = 1;
        float vel_t_ = 0;
        float interpolation_leading_joint_max_speed_ratio_ = 0.0f;
        InterpolationMethod inter_method_ = InterpolationMethod::kCos;
        InterpolatorFactory motor_inter_;
        InterpolatorFactory joint_inter_;
        InterpolatorFactory task_inter_;

        //Waypoint related
        PanelTargetMode arm_target_mode_ = PanelTargetMode::kSinglePoint;
        std::vector<VectorXd> arm_waypoint_cmd_;
        std::vector<float> arm_waypoint_time_;
        std::vector<VectorXd> arm_waypoint_leading_joint_max_vel_;
        VectorXd target_before_interpolation_;

        //Recording & Playback
        const float kMaxRecordDurationMinute_ = 10;
        int max_record_trajectory_size_;
        bool max_record_size_reached_ = false;
        bool enable_recording_ = false;
        int playback_index_ = 0;
        PlaybackState prev_playback_cmd_ = PlaybackState::kStop;
        PlaybackState playback_cmd_ = PlaybackState::kStop;
        std::vector<VectorXd> record_trajectory_;
        std::vector<VectorXd> playback_trajectory_;
        bool playback_switched_cmd_ = false;
};
#endif