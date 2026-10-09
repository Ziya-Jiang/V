#include "sole_arm_planner.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <stdlib.h>
#include <math.h>
#include <config_loader.h>

float smoothing_factor(float t_e, float cutoff){
    float r = static_cast<float>(2 * M_PI * cutoff * t_e);
    return r / (r + 1);
}
float exponential_smoothing(float a, float x, float x_prev){
    return a * x + (1 - a) * x_prev;
}

SoleArmPlanner::SoleArmPlanner(PlannerCommand& pl_cmd, PlannerState& pl_state, 
                                float dt, const YAML::Node& config, const std::string& config_prefix_path, const std::string& urdf_path,
                                const std::string& library_name, int arm_i): planner_command_(pl_cmd), 
                                planner_state_(pl_state), arm_i_(arm_i), dt_(dt){
    YAML::Node arm_config = config["arm"][arm_i];
    joint_size_ = arm_config["joint_size"].as<int>();
    joint_pos_limits_ = Matrix2Xf::Zero(2, joint_size_);
    joint_vel_limits_ = Matrix2Xf::Zero(2, joint_size_);
    joint_acc_limits_ = Matrix2Xf::Zero(2, joint_size_);
    joint_tor_limits_ = Matrix2Xf::Zero(2, joint_size_);
    joint_jump_pos_limits_ = VectorXf::Zero(joint_size_);
    joint_jump_vel_limits_ = VectorXf::Zero(joint_size_);
    joint_jump_tor_limits_ = VectorXf::Zero(joint_size_);
    task_motion_limit_linear_velocity_ = Vector3f::Zero();
    task_motion_limit_angular_velocity_ = Vector3f::Zero();

    pos_vel_kp_ = VectorXd::Zero(joint_size_);
    pos_tor_kp_ = VectorXd::Zero(joint_size_);
    vel_tor_kp_ = VectorXd::Zero(joint_size_);
    pos_vel_kd_ = VectorXd::Zero(joint_size_);
    pos_tor_kd_ = VectorXd::Zero(joint_size_);
    vel_tor_kd_ = VectorXd::Zero(joint_size_);

    sliced_joint_tar_ = VectorXd::Zero(joint_size_);
    sliced_null_tar_  = VectorXd::Zero(joint_size_);
    sliced_motor_tar_ = VectorXd::Zero(joint_size_);

    // Initial to Home posiiton
    motor_tar_      = VectorXd::Zero(joint_size_);
    joint_cmd_      = VectorXd::Zero(joint_size_);
    motor_cmd_      = VectorXd::Zero(joint_size_);
    task_cmd_       = Vector6d::Zero();
    pid_cmd_        = VectorXd::Zero(joint_size_);
    motor_cmd_final_= VectorXd::Zero(joint_size_);
    joint_jump_last_joint_cmd_ = VectorXd::Zero(joint_size_);

    filtered_motor_tor_        = VectorXd::Zero(joint_size_); 
    filtered_joint_tor_        = VectorXd::Zero(joint_size_);      
    filtered_motor_tor_kalman_ = VectorXd::Zero(joint_size_);
    filtered_joint_tor_kalman_ = VectorXd::Zero(joint_size_);
    filtered_motor_tor_euro_   = VectorXd::Zero(joint_size_);
    filtered_joint_tor_euro_   = VectorXd::Zero(joint_size_);
    euro_x_prev_               = VectorXd::Zero(joint_size_);
    euro_dx_prev_              = VectorXd::Zero(joint_size_);

    joint_tor_gravity_     = VectorXd::Zero(joint_size_);
    motor_tor_gravity_     = VectorXd::Zero(joint_size_);
    motor_tor_offset_      = VectorXd::Zero(joint_size_);
    motor_vel_offset_      = VectorXd::Zero(joint_size_);
    motor_pos_             = VectorXd::Zero(joint_size_);
    motor_vel_             = VectorXd::Zero(joint_size_);
    motor_acc_             = VectorXd::Zero(joint_size_);
    motor_tor_             = VectorXd::Zero(joint_size_);
    joint_pos_             = VectorXd::Zero(joint_size_);
    joint_vel_             = VectorXd::Zero(joint_size_);
    joint_acc_             = VectorXd::Zero(joint_size_);
    joint_tor_             = VectorXd::Zero(joint_size_);
    last_joint_pos_        = VectorXd::Zero(joint_size_);
    last_joint_vel_        = VectorXd::Zero(joint_size_);
    last_motor_vel_        = VectorXd::Zero(joint_size_);
    task_base_joint_pos_   = VectorXd::Zero(joint_size_);
    task_base_end_pose_    = VectorXd::Zero(joint_size_);

    limited_joint_vel_  = VectorXd::Zero(joint_size_);
    limited_joint_tor_  = VectorXd::Zero(joint_size_);
    reached_limit_ji_   = VectorXi::Zero(joint_size_);
    last_sat_joint_cmd_ = VectorXd::Zero(joint_size_);
    last_task_solved_joint_cmd_ = VectorXd::Zero(joint_size_);
    
    joint_impedance_target_ = VectorXd::Zero(joint_size_);
    admittance_last_cmd_    = VectorXd::Zero(joint_size_);

    ee_pose_   = Vector7d::Zero();
    tool_pose_ = Vector7d::Zero();

    motor_inter_ = InterpolatorFactory(inter_method_, dt_, joint_size_, "arm_"+std::to_string(arm_i_)+"_motor");
    joint_inter_ = InterpolatorFactory(inter_method_, dt_, joint_size_, "arm_"+std::to_string(arm_i_)+"_joint");
    task_inter_ = InterpolatorFactory(inter_method_, dt_, 6, "arm_"+std::to_string(arm_i_)+"_task");
    
    //Recording & Playback
    max_record_trajectory_size_ = static_cast<int>(std::ceil(kMaxRecordDurationMinute_*60 / dt_));
    record_trajectory_.reserve(max_record_trajectory_size_);

    // printYamlNode(arm_config);
    for (int i =0; i<joint_size_; i++){
        pos_vel_kp_[i] = (double)arm_config["pid"]["pos_vel_kp"][i].as<float>();
        pos_tor_kp_[i] = (double)arm_config["pid"]["pos_tor_kp"][i].as<float>();
        vel_tor_kp_[i] = (double)arm_config["pid"]["vel_tor_kp"][i].as<float>();
        pos_vel_kd_[i] = (double)arm_config["pid"]["pos_vel_kd"][i].as<float>();
        pos_tor_kd_[i] = (double)arm_config["pid"]["pos_tor_kd"][i].as<float>();
        vel_tor_kd_[i] = (double)arm_config["pid"]["vel_tor_kd"][i].as<float>();
    }

    const float kDegToRad = static_cast<float>(M_PI/180.0f);
    for (int i =0; i <joint_size_; i++){
        joint_pos_limits_(0,i) = arm_config["safety"]["joint_soft_limit"]["position"][0][i].as<float>() * kDegToRad;
        joint_pos_limits_(1,i) = arm_config["safety"]["joint_soft_limit"]["position"][1][i].as<float>() * kDegToRad;
        joint_vel_limits_(0,i) = arm_config["safety"]["joint_soft_limit"]["velocity"][i].as<float>() * kDegToRad * -1.0f;
        joint_vel_limits_(1,i) = arm_config["safety"]["joint_soft_limit"]["velocity"][i].as<float>() * kDegToRad;
        joint_acc_limits_(0,i) = arm_config["safety"]["joint_soft_limit"]["acceleration"][i].as<float>() * kDegToRad * -1.0f;
        joint_acc_limits_(1,i) = arm_config["safety"]["joint_soft_limit"]["acceleration"][i].as<float>() * kDegToRad;
        joint_tor_limits_(0,i) = arm_config["safety"]["joint_soft_limit"]["torque"][i].as<float>() * -1.0f;
        joint_tor_limits_(1,i) = arm_config["safety"]["joint_soft_limit"]["torque"][i].as<float>();
        joint_jump_pos_limits_(i) = arm_config["safety"]["joint_jump"]["position"][i].as<float>() * kDegToRad * dt_;
        joint_jump_vel_limits_(i) = arm_config["safety"]["joint_jump"]["velocity"][i].as<float>() * kDegToRad * dt_;
        joint_jump_tor_limits_(i) = arm_config["safety"]["joint_jump"]["torque"][i].as<float>() * dt_;
    }

    for (int i =0; i <3; i++){
        task_motion_limit_linear_velocity_(i) = arm_config["safety"]["task_motion_limit"]["linear_velocity"][i].as<float>();
        task_motion_limit_angular_velocity_(i) = arm_config["safety"]["task_motion_limit"]["angular_velocity"][i].as<float>() * static_cast<float>(M_PI/180.0f);
    }

    std::string robot_type = arm_config["type"].as<std::string>();
    std::vector<std::string> joint_names = arm_config["urdf_joints"].as<std::vector<std::string>>();
    std::vector<std::string> link_names  = arm_config["urdf_links"].as<std::vector<std::string>>();
    Vector3d tool_offset = Vector3d::Zero();
    if (config["gripper"] && config["gripper"].size() > arm_i_){
        YAML::Node offset_node = config["gripper"][arm_i_]["offset_from_ee"];
        tool_offset << offset_node[0].as<float>(), offset_node[1].as<float>(), offset_node[2].as<float>();
    }
    std::string ee_link = link_names[link_names.size()-1];
    robotics_ = new Robotics(urdf_path, joint_names, ee_link, tool_offset, arm_i_, joint_size_, 
        joint_pos_limits_, library_name);
    VectorXf gear_ratio(joint_size_);
    robotics_->setArm(robot_type, gear_ratio);
        
    //Gravity comp
    VectorXd gravity_ratio(joint_size_);
    VectorXd gravity_kd(joint_size_);
    for (int i =0; i<joint_size_; i++){
        gravity_ratio[i] = arm_config["force_control"]["gravity"]["ratio"][i].as<float>();
        gravity_kd[i] = arm_config["force_control"]["gravity"]["kd"][i].as<float>();
    }
    robotics_->setGravityGains(gravity_ratio, gravity_kd);

    //Gravity with boundary
    VectorXd gravity_with_boundary_joint_saturation = VectorXd::Zero(joint_size_);
    Vector3d gravity_with_boundary_kp = Vector3d::Zero();
    Vector3d gravity_with_boundary_boundary = Vector3d::Zero();
    robotics_->setGravityWithBoundary(gravity_with_boundary_kp, gravity_with_boundary_boundary, gravity_with_boundary_joint_saturation);

    //Genral admittance config
    admittance_force_upper_bound_ = 0;
    admittance_threshold_ = 0;
    admittance_max_speed_ = 0;
    
    //Task admittance config
    float task_admittance_kp = 0;
    robotics_->setTaskAdmittance(task_admittance_kp);

    //Joint admittance config
    float joint_admittance_damping;
    VectorXd joint_admittance_force_upper_bound = VectorXd::Zero(joint_size_);
    VectorXd joint_admittance_threshold = VectorXd::Zero(joint_size_);
    VectorXd joint_admittance_max_speed = VectorXd::Zero(joint_size_);
    VectorXd joint_admittance_max_acceleration = VectorXd::Zero(joint_size_);
    VectorXd joint_admittance_extern_force_k = VectorXd::Zero(joint_size_);
    joint_admittance_damping = 0;
    robotics_->setJointAdmittance(joint_admittance_damping, joint_admittance_force_upper_bound, joint_admittance_threshold, joint_admittance_max_speed, joint_admittance_max_acceleration, joint_admittance_extern_force_k);

    //Null space admittance config
    float null_admittance_kp = 0;
    float null_admittance_kd = 0;
    float null_admittance_threshold = 0;
    float null_admittance_saturation = 0;
    int null_admittance_ji=0;
    robotics_->setNullAdmittance(null_admittance_kp, null_admittance_kd, null_admittance_threshold, null_admittance_saturation, null_admittance_ji);
    
    //Task impedance config
    Vector6d tool_force_kp = VectorXd::Zero(6);
    Vector6d tool_force_kd = VectorXd::Zero(6);
    Vector6d tool_force_ki = VectorXd::Zero(6);
    VectorXd tool_force_joint_saturation = VectorXd::Zero(joint_size_);
    VectorXd tool_force_joint_torque_ratio = VectorXd::Zero(joint_size_);
    VectorXd tool_force_joint_additional_kd = VectorXd::Zero(joint_size_);
    VectorXd task_null_k = VectorXd::Zero(joint_size_);
    float ki_ratio = 0;
    robotics_->setTaskImepdance(tool_force_kp, tool_force_kd, tool_force_ki, ki_ratio, tool_force_joint_saturation, tool_force_joint_torque_ratio, tool_force_joint_additional_kd, task_null_k);
    
    //Joint impedance config
    VectorXd joint_force_kp(joint_size_);
    VectorXd joint_force_kd(joint_size_);
    VectorXd joint_force_saturation(joint_size_);
    for (int i =0; i<joint_size_; i++){
        joint_force_kp[i] = arm_config["force_control"]["joint_impedance"]["kp"][i].as<float>();
        joint_force_kd[i] = arm_config["force_control"]["joint_impedance"]["kd"][i].as<float>();
        joint_force_saturation[i] = arm_config["force_control"]["joint_impedance"]["torque_saturation"][i].as<float>();
    }
    robotics_->setJointImpedance(joint_force_kp, joint_force_kd, VectorXd::Zero(joint_size_));
    robotics_->setJointSaturation(joint_force_saturation);


    //Friction config
    VectorXd vel_filter_ratio(joint_size_);
    VectorXd friction_ratio(joint_size_);
    VectorXd friction_coulumb(joint_size_);
    VectorXd friction_viscous(joint_size_);
    for (int i =0; i<joint_size_; i++){
        vel_filter_ratio[i] = arm_config["force_control"]["velocity_low_pass_filter_ratio"][i].as<float>();
        friction_ratio[i] = arm_config["force_control"]["friction"]["coulumb_with_viscous"]["ratio"][i].as<float>();
        friction_coulumb[i] = arm_config["force_control"]["friction"]["coulumb_with_viscous"]["coulumb"][i].as<float>();
        friction_viscous[i] = arm_config["force_control"]["friction"]["coulumb_with_viscous"]["viscous"][i].as<float>();
    }
    robotics_->setJointVelLowPassFilterRatio(vel_filter_ratio);
    robotics_->setFrictionAsCoulumbWithViscous(friction_ratio, friction_coulumb, friction_viscous);

    // Initialize planner state and command
    planner_state_.JointSize[arm_i_] = joint_size_;
    planner_command_.JointSize[arm_i_] = joint_size_;
    setJointVal(joint_cmd_);
    computeFK(joint_pos_); //To initialize pose
    for (int i =0; i< joint_size_; i++){
        planner_state_.JointPos[arm_i_][i] = 0.0f;
        planner_state_.JointVel[arm_i_][i] = 0.0f;
        planner_state_.JointTor[arm_i_][i] = 0.0f;
        planner_state_.MotorPos[arm_i_][i] = 0.0f;
        planner_state_.MotorVel[arm_i_][i] = 0.0f;
        planner_state_.MotorTor[arm_i_][i] = 0.0f;
        planner_command_.Target[arm_i_][i] = 0.0f;
        
        VectorXd row = joint_pose_.row(i);
        for (int j=0; j<7; j++){
            planner_state_.JointPose[arm_i_][i][j] = static_cast<float>(row[j]);
        }
    }
    for (int j=0; j<7; j++){
        planner_state_.EEPose[arm_i_][j] = (float)ee_pose_[j];
        planner_state_.ToolPose[arm_i_][j] = (float)tool_pose_[j];
    }

    //Update config setting to planner state
    planner_state_.tool_offset[arm_i_][0] = (float)tool_offset[0];
    planner_state_.tool_offset[arm_i_][1] = (float)tool_offset[1];
    planner_state_.tool_offset[arm_i_][2] = (float)tool_offset[2];
    for (int i =0; i<joint_size_; i++){
        planner_state_.arm_soft_limit_position[arm_i_][i][0] = (float)joint_pos_limits_(0,i);
        planner_state_.arm_soft_limit_position[arm_i_][i][1] = (float)joint_pos_limits_(1,i);
        planner_state_.arm_soft_limit_velocity[arm_i_][i][0] = (float)joint_vel_limits_(0,i);
        planner_state_.arm_soft_limit_velocity[arm_i_][i][1] = (float)joint_vel_limits_(1,i);
        planner_state_.arm_soft_limit_torque[arm_i_][i][0] = (float)joint_tor_limits_(0,i);
        planner_state_.arm_soft_limit_torque[arm_i_][i][1] = (float)joint_tor_limits_(1,i);

        planner_state_.arm_hard_limit_position[arm_i_][i][0] = arm_config["safety"]["joint_hard_limit"]["position"][0][i].as<float>() * kDegToRad;
        planner_state_.arm_hard_limit_position[arm_i_][i][1] = arm_config["safety"]["joint_hard_limit"]["position"][1][i].as<float>() * kDegToRad;
        planner_state_.arm_hard_limit_velocity[arm_i_][i][0] = arm_config["safety"]["joint_hard_limit"]["velocity"][i].as<float>() * kDegToRad * -1.0f;
        planner_state_.arm_hard_limit_velocity[arm_i_][i][1] = arm_config["safety"]["joint_hard_limit"]["velocity"][i].as<float>() * kDegToRad;
        planner_state_.arm_hard_limit_torque[arm_i_][i][0] = arm_config["safety"]["joint_hard_limit"]["torque"][i].as<float>() * -1.0f;
        planner_state_.arm_hard_limit_torque[arm_i_][i][1] = arm_config["safety"]["joint_hard_limit"]["torque"][i].as<float>();

        planner_state_.arm_follow_limit_position[arm_i_][i] = arm_config["safety"]["joint_follow"]["position"][i].as<float>() * kDegToRad;
        planner_state_.arm_follow_limit_velocity[arm_i_][i] = arm_config["safety"]["joint_follow"]["velocity"][i].as<float>() * kDegToRad;
        planner_state_.arm_follow_limit_torque[arm_i_][i] = arm_config["safety"]["joint_follow"]["torque"][i].as<float>();

        planner_state_.arm_jump_limit_position[arm_i_][i] = arm_config["safety"]["joint_jump"]["position"][i].as<float>() * kDegToRad;
        planner_state_.arm_jump_limit_velocity[arm_i_][i] = arm_config["safety"]["joint_jump"]["velocity"][i].as<float>() * kDegToRad;
        planner_state_.arm_jump_limit_torque[arm_i_][i] = arm_config["safety"]["joint_jump"]["torque"][i].as<float>();
    }
}

SoleArmPlanner::~SoleArmPlanner(){
    delete robotics_;
    robotics_ = nullptr;
}

MotionControl SoleArmPlanner::getLastMotionType(){
    return motion_type_;
}

// Update the simulation and motor command upond timeout
void SoleArmPlanner::update(const PanelCommand& pa_cmd, const RobotState& ro_state){
    if (pa_cmd.sequence_id < sequence_id_) return;  //If the received command is old, ignore the update to avoid state jump back
    
    long t0 = get_time_now();
    MotionControl prev_motion_type = motion_type_;
    ControlType   prev_target_type_ = target_type_;
    prev_playback_cmd_ = playback_cmd_;
    if (pa_cmd.sequence_id > sequence_id_){
        planner_command_.connection_state = pa_cmd.connection_state;
        planner_command_.enable_set = !pa_cmd.need_setting_update;
        is_simulation_ = pa_cmd.simulation;
        motion_type_ = pa_cmd.motion_type;
        task_orient_type_ = pa_cmd.task_orien_type;
        target_type_ = pa_cmd.target_type;
        actuator_mode_ = pa_cmd.actuator_mode;

        //Update interpolation method
        if ((arm_target_mode_ == PanelTargetMode::kSinglePoint && inter_method_ != pa_cmd.interpolation_type) ||
            acc_t_ != pa_cmd.InterpolationAccTime ||
            vel_t_ != pa_cmd.InterpolationConstVelTime ||
            interpolation_leading_joint_max_speed_ratio_ != pa_cmd.interpolation_speed_ratio ||
            pa_cmd.reset_interpolation ||
            pa_cmd.arm_target_mode != arm_target_mode_ ||
            prev_motion_type != motion_type_ ||
            prev_target_type_ != target_type_){
            arm_target_mode_ = pa_cmd.arm_target_mode;
            VectorXd leading_joint_max_vel = VectorXd::Zero(joint_size_);
            if (arm_target_mode_ == PanelTargetMode::kWaypoint){
                inter_method_ = InterpolationMethod::kQuinticPath;
            }else if (arm_target_mode_ == PanelTargetMode::kRawSinglePoint){
                inter_method_ = InterpolationMethod::kNone;
            }else{
                inter_method_ = pa_cmd.interpolation_type;
            }
            
            acc_t_ = pa_cmd.InterpolationAccTime;
            vel_t_ = pa_cmd.InterpolationConstVelTime;
            //Update leading joint max speed ratio
            interpolation_leading_joint_max_speed_ratio_ = pa_cmd.interpolation_speed_ratio;
            float clamp_ratio = std::clamp(interpolation_leading_joint_max_speed_ratio_, 0.0f, 1.0f);
            if (acc_t_ > 0 && clamp_ratio == 0){ //If invalid ratio, use max speed ratio to validate duration
                clamp_ratio = 1.0f;
            }

            //Update leading joint max speed
            if (motion_type_ != MotionControl::kTaskLine){
                for (int i = 0; i < joint_size_; i++){
                    if (target_type_ == ControlType::kPosition){
                        leading_joint_max_vel[i] = joint_vel_limits_(1,i) * clamp_ratio;
                    }else if (target_type_ == ControlType::kVelocity){
                        leading_joint_max_vel[i] = joint_acc_limits_(1,i) * clamp_ratio;
                    }
                }
            }else if (motion_type_ == MotionControl::kTaskLine){
                leading_joint_max_vel[0] = task_motion_limit_linear_velocity_(0) * clamp_ratio;
                leading_joint_max_vel[1] = task_motion_limit_linear_velocity_(1) * clamp_ratio;
                leading_joint_max_vel[2] = task_motion_limit_linear_velocity_(2) * clamp_ratio;
                leading_joint_max_vel[3] = task_motion_limit_angular_velocity_(0) * clamp_ratio;
                leading_joint_max_vel[4] = task_motion_limit_angular_velocity_(1) * clamp_ratio;
                leading_joint_max_vel[5] = task_motion_limit_angular_velocity_(2) * clamp_ratio;
            }
            
            switch (motion_type_){
                case MotionControl::kJoint:
                case MotionControl::kControlAlgorithm:
                case MotionControl::kPlayback:
                    joint_inter_.updateParam((InterpolationMethod)inter_method_, acc_t_, vel_t_, leading_joint_max_vel);
                    break;
                case MotionControl::kTask: //We use joint interpolation for task control
                    joint_inter_.updateParam((InterpolationMethod)inter_method_, acc_t_, vel_t_, leading_joint_max_vel);
                    break;
                case MotionControl::kTaskLine:
                    task_inter_.updateParam((InterpolationMethod)inter_method_, acc_t_, vel_t_, leading_joint_max_vel);
                    break;
                case MotionControl::kMotor:
                    motor_inter_.updateParam((InterpolationMethod)inter_method_, acc_t_, vel_t_, leading_joint_max_vel);
                    break;
                default:
                    break;
            }
        }

        //Read panel waypoint cmd
        if (pa_cmd.arm_target_mode == PanelTargetMode::kWaypoint){
            bool valid_waypoint_time = true;
            bool valid_waypoint_ratio = true;
            arm_waypoint_cmd_.clear();
            arm_waypoint_time_.clear();
            arm_waypoint_leading_joint_max_vel_.clear();
            last_task_waypoint_res_.clear();
            last_task_waypoint_cmd_.clear();
            for (int i = 0; i < pa_cmd.ArmWaypointSize; ++i) {
                Eigen::VectorXd row(joint_size_);
                for (int j = 0; j < joint_size_; ++j) {
                    row(j) = pa_cmd.ArmWaypointCmd[i][arm_i_][j];
                }
                
                arm_waypoint_cmd_.push_back(row);
                arm_waypoint_time_.push_back(pa_cmd.ArmWaypointInterpolationTime[i]);
                valid_waypoint_time &= (pa_cmd.ArmWaypointInterpolationTime[i] > 0);
            }
            if (!valid_waypoint_time) arm_waypoint_time_.clear();


            for (int i = 0; i < pa_cmd.ArmWaypointSize; ++i) {
                //Update leading joint max speed ratio
                float clamp_ratio = std::clamp(pa_cmd.ArmWaypointInterpolationSpeedRatio[i], 0.0f, 1.0f);
                if (valid_waypoint_ratio && clamp_ratio == 0){ //If invalid ratio, use max speed ratio to validate duration
                    clamp_ratio = 1.0f;
                }

                if (motion_type_ != MotionControl::kTaskLine){
                    Eigen::VectorXd leading_joint_max_vel = VectorXd::Zero(joint_size_);
                    for (int i = 0; i < joint_size_; i++){
                        if (target_type_ == ControlType::kPosition){
                            leading_joint_max_vel[i] = joint_vel_limits_(1,i) * clamp_ratio;
                        }else if (target_type_ == ControlType::kVelocity){
                            leading_joint_max_vel[i] = joint_acc_limits_(1,i) * clamp_ratio;
                        }
                    }
                    arm_waypoint_leading_joint_max_vel_.push_back(leading_joint_max_vel);
                }else if (motion_type_ == MotionControl::kTaskLine){
                    Eigen::VectorXd leading_joint_max_vel = VectorXd::Zero(6);
                    leading_joint_max_vel[0] = task_motion_limit_linear_velocity_(0) * clamp_ratio;
                    leading_joint_max_vel[1] = task_motion_limit_linear_velocity_(1) * clamp_ratio;
                    leading_joint_max_vel[2] = task_motion_limit_linear_velocity_(2) * clamp_ratio;
                    leading_joint_max_vel[3] = task_motion_limit_angular_velocity_(0) * clamp_ratio;
                    leading_joint_max_vel[4] = task_motion_limit_angular_velocity_(1) * clamp_ratio;
                    leading_joint_max_vel[5] = task_motion_limit_angular_velocity_(2) * clamp_ratio;
                    arm_waypoint_leading_joint_max_vel_.push_back(leading_joint_max_vel);
                }
                if (!valid_waypoint_time & !valid_waypoint_ratio) break;
            }
        }

        //Force Control
        if (motion_type_ == MotionControl::kControlAlgorithm){
            if (control_algorithm_ != ControlAlgorithm::kGravityWithBoundary && pa_cmd.control_algorithm == ControlAlgorithm::kGravityWithBoundary){
                setGravityWithBoundaryTarget();
            }

            if ((control_algorithm_ != ControlAlgorithm::kTaskImpedance && pa_cmd.control_algorithm == ControlAlgorithm::kTaskImpedance) ||
            (control_algorithm_ != ControlAlgorithm::kJointImpedance && pa_cmd.control_algorithm == ControlAlgorithm::kJointImpedance)){
                setImpedanceTarget();
            }
            
            if ((control_algorithm_ != ControlAlgorithm::kTaskAdmittance && pa_cmd.control_algorithm == ControlAlgorithm::kTaskAdmittance) ||
            (control_algorithm_ != ControlAlgorithm::kJointAdmittance && pa_cmd.control_algorithm == ControlAlgorithm::kJointAdmittance) ||
            (control_algorithm_ != ControlAlgorithm::kNullAdmittance && pa_cmd.control_algorithm == ControlAlgorithm::kNullAdmittance)){
                admittance_last_cmd_ = VectorXd::Zero(joint_size_);
                if (pa_cmd.control_algorithm == ControlAlgorithm::kNullAdmittance)
                    resetControlMem(MotionControl::kNull);
            }
            control_algorithm_ = pa_cmd.control_algorithm;
        }else{
            control_algorithm_ = ControlAlgorithm::kNone;
        }

        //Reset reference position for task & null control
        if (pa_cmd.reset_control_mem || (prev_motion_type != motion_type_ && motion_type_!=MotionControl::kJoint)){
            resetControlMem(motion_type_);
        }
        
        //Update config setting
        Vector3d current_tool_offset = robotics_->getToolOffset();
        if (current_tool_offset[0] != pa_cmd.tool_offset[arm_i_][0] || 
            current_tool_offset[1] != pa_cmd.tool_offset[arm_i_][1] || 
            current_tool_offset[2] != pa_cmd.tool_offset[arm_i_][2]) { 
            Vector3d tool_offset(pa_cmd.tool_offset[arm_i_][0], pa_cmd.tool_offset[arm_i_][1], pa_cmd.tool_offset[arm_i_][2]);
            robotics_->setToolOffset(tool_offset);
            planner_state_.tool_offset[arm_i_][0] = (float)tool_offset[0];
            planner_state_.tool_offset[arm_i_][1] = (float)tool_offset[1];
            planner_state_.tool_offset[arm_i_][2] = (float)tool_offset[2];
        }
        for (int joint_i =0; joint_i<joint_size_; joint_i++){
            joint_pos_limits_(0,joint_i) = pa_cmd.arm_soft_limit_position[arm_i_][joint_i][0];
            joint_pos_limits_(1,joint_i) = pa_cmd.arm_soft_limit_position[arm_i_][joint_i][1];
            joint_vel_limits_(0,joint_i) = pa_cmd.arm_soft_limit_velocity[arm_i_][joint_i][0];
            joint_vel_limits_(1,joint_i) = pa_cmd.arm_soft_limit_velocity[arm_i_][joint_i][1];
        }

        //Recording & Playback
        if (motion_type_ == MotionControl::kPlayback){
            if (prev_motion_type != MotionControl::kPlayback){
                if (target_type_ != ControlType::kPosition){
                    std::cerr << "Playback control require target type set to ControlType::kPosition" << std::endl;
                    return;
                }
                playback_index_ = 0;
            }
            playback_cmd_ = pa_cmd.playback_cmd;
            sliced_joint_tar_ = joint_pos_;
            if (playback_cmd_ != prev_playback_cmd_) playback_switched_cmd_ = true;
        }else{
            playback_cmd_ = PlaybackState::kStop;
        }

        if (pa_cmd.enable_recording && !enable_recording_ && motion_type_ != MotionControl::kPlayback){
            record_trajectory_.clear();
            max_record_size_reached_ = false;
        }
        enable_recording_ = pa_cmd.enable_recording;

        //Read panel cmd
        for (int i =0; i<joint_size_; i++){
            joint_cmd_[i] = (double)pa_cmd.JointCmd[arm_i_][i];
            motor_cmd_[i] = (double)pa_cmd.MotorCmd[arm_i_][i];
            if (i < 6)
                task_cmd_[i] = (double)pa_cmd.TaskCmd[arm_i_][i];
        }
    }
    sequence_id_ = pa_cmd.sequence_id;

    //Clear flag & state
    planner_state_.system_diagnostic_flags = DiagnosticFlags::kNone;
    for (int i = 0; i < joint_size_; i++){
        planner_state_.arm_joint_diagnostic_flags[arm_i_][i] = DiagnosticFlags::kNone;
    }
    if (prev_motion_type != motion_type_ ){
        planner_state_.plan_result = PlanResult::kSuccess;
    }

    planner_state_.setup_cost = get_time_now() - t0;

    if (planner_command_.enable_set){
        //Calculate Dynamics
        computeID(); 

        planner_state_.ID_cost = get_time_now() - t0;
        planner_state_.Null_cost = 0;
        planner_state_.IK_cost = 0;
        
        //PID Control
        try{
            switch (motion_type_){
                case MotionControl::kJoint:
                case MotionControl::kControlAlgorithm:
                    setJointVal(joint_cmd_);
                    break;
                case MotionControl::kTaskLine:
                    setTaskLineVal(task_cmd_, LineMoveStrategy::kRejectEntirely);
                    break;
                case MotionControl::kTask:
                    setTaskVal(task_cmd_);
                    break;
                case MotionControl::kPlayback:
                    setPlaybackVal();
                    break;
                // case MotionControl::kNull:
                //     setNullVal(joint_cmd_);
                //     break;
                // case MotionControl::kMotor:
                //     setMotorVal(motor_cmd_);
                //     break;
                default:
                    break;
            }
        }catch(const std::exception& e){
            std::cerr << e.what() << std::endl;
            return;
        }

        bool isTor = (actuator_mode_ == ControlType::kTorque);
        VectorXd pid_joint_cmd = robotics_->convertMotorJoint(pid_cmd_, ConvertType::kJoint, isTor);

        //Ensure joint position within limits under Velocity/Torque control
        VectorXd guard_pos_cmd = guardJointPositionLimits(pid_joint_cmd);

        //Saturate command on joint position/velocity/touque limit
        VectorXd sat_cmd = saturate(guard_pos_cmd, false); // Saturation is on Joint level

        //Prevent large step changes in joint commands
        VectorXd jump_cmd = limitJointJump(sat_cmd, false);
        motor_cmd_final_ = robotics_->convertMotorJoint(jump_cmd, ConvertType::kMotor, isTor);
    }else{
        joint_jump_last_actuator_mode_ = ControlType::kInvalid;
        motor_cmd_final_ = (actuator_mode_ == ControlType::kPosition)? motor_pos_ : VectorXd::Zero(joint_size_);
        planner_state_.system_state = SystemState::kSettling;
    }

    //Update PlannerCommand
    if (is_simulation_){
        planner_command_.command_mode = ControlType::kVelocity;
        updateRobotCmdTarget(VectorXd::Zero(joint_size_));
    }else{
        planner_command_.command_mode = static_cast<ControlType>(actuator_mode_);
        updateRobotCmdTarget(motor_cmd_final_);
    }
    planner_state_.simulation_addition_cost = get_time_now() - t0;

    //Update PlannerState 
    const InterpolatorFactory* current_interpolator_ptr = nullptr;
    for (int i=0; i < joint_size_; i++){
        switch (motion_type_){
            case MotionControl::kJoint: 
            case MotionControl::kControlAlgorithm:
            case MotionControl::kPlayback:
                planner_state_.interpolated_target[arm_i_][i] = (float)sliced_joint_tar_[i];
                planner_state_.target_before_interpolation[arm_i_][i] = (float)target_before_interpolation_[i];
                current_interpolator_ptr = &joint_inter_;
                break;
            case MotionControl::kTask: 
                planner_state_.interpolated_target[arm_i_][i] = (float)sliced_joint_tar_[i]; 
                // planner_state_.target_before_interpolation[arm_i_][i] = (float)target_before_interpolation_[i];
                if (i < 3){
                    planner_state_.target_before_interpolation[arm_i_][i] = static_cast<float>(target_before_interpolation_[i] + task_base_end_pose_.head(3)[i]);
                }else if (i < 6){
                    planner_state_.target_before_interpolation[arm_i_][i] = static_cast<float>(target_before_interpolation_[i] + task_base_rpy_[i-3]);
                }
                current_interpolator_ptr = &joint_inter_;
                break;
            case MotionControl::kTaskLine: 
                if (i < 3){
                    planner_state_.interpolated_target[arm_i_][i] = static_cast<float>(sliced_task_tar_[i] + task_base_end_pose_.head(3)[i]);
                    planner_state_.target_before_interpolation[arm_i_][i] = static_cast<float>(target_before_interpolation_[i] + task_base_end_pose_.head(3)[i]);
                }else if (i < 6){
                    planner_state_.interpolated_target[arm_i_][i] = static_cast<float>(sliced_task_tar_[i] + task_base_rpy_[i-3]);
                    planner_state_.target_before_interpolation[arm_i_][i] = static_cast<float>(target_before_interpolation_[i] + task_base_rpy_[i-3]);
                }
                current_interpolator_ptr = &task_inter_;
                break;
            case MotionControl::kMotor: 
                planner_state_.interpolated_target[arm_i_][i] = (float)sliced_motor_tar_[i];
                current_interpolator_ptr = &motor_inter_;
                break; 
            default: break;
        }
        planner_state_.sat_motor_cmd_[arm_i_][i] = static_cast<float>(motor_cmd_final_[i]);
    }

    if (motion_type_ != MotionControl::kPlayback && motion_type_ != MotionControl::kControlAlgorithm){
        if (current_interpolator_ptr && current_interpolator_ptr->getState() != InterpolationMotionPhase::kFinished && 
            current_interpolator_ptr->getState() != InterpolationMotionPhase::kInitialized &&
            current_interpolator_ptr->getState() != InterpolationMotionPhase::kInterrupted){
            planner_state_.system_state = SystemState::kMoving;
        }else if (motor_vel_.norm() < 0.01){
            planner_state_.system_state = SystemState::kIdle;
        }else if (target_type_ == ControlType::kPosition){
            planner_state_.system_state = SystemState::kSettling;
        }
    }else if (motion_type_ == MotionControl::kPlayback){
        if (playback_trajectory_.size() == 0 || playback_index_ == playback_trajectory_.size()-1 || playback_cmd_ == PlaybackState::kStop){
            if (motor_vel_.norm() < 0.01){
                planner_state_.system_state = SystemState::kIdle;
            }else{
                planner_state_.system_state = SystemState::kSettling;
            }
        }else{
            planner_state_.system_state = SystemState::kMoving;
        }
    }else if (motion_type_ == MotionControl::kControlAlgorithm){
        if (motor_vel_.norm() < 0.01){
            planner_state_.system_state = SystemState::kIdle;
        }else{
            planner_state_.system_state = SystemState::kMoving;
        }
    }
    planner_state_.received_sequence_id = sequence_id_;
}

void SoleArmPlanner::computeFilteredTorque(){
        //Simple filter
        float filter_k = 0.96f;
        filtered_motor_tor_ = filter_k * filtered_motor_tor_ + (1 - filter_k) * motor_tor_;
        filtered_joint_tor_ = robotics_->convertMotorJoint(filtered_motor_tor_, ConvertType::kJoint, true);

        //Kalman Filter
        float a = 1, b = 0, h = 1, r = 0.05f, q = 0.002f;
        filtered_motor_tor_kalman_ = a * filtered_motor_tor_kalman_ + b * motor_tor_;
        kalman_p_ = a * kalman_p_ * a + q;
        kalman_g_ = kalman_p_ * h /(h * kalman_p_ * h + r);
        filtered_motor_tor_kalman_ = filtered_motor_tor_kalman_ + kalman_g_ * (motor_tor_ - h * filtered_motor_tor_kalman_);
        kalman_p_ = (1 - kalman_g_ * h) * kalman_p_;
        filtered_joint_tor_kalman_ = robotics_->convertMotorJoint(filtered_motor_tor_kalman_, ConvertType::kJoint, true);

        //One Euro Filter
        float min_cutoff = 1.0,  beta = 0, d_cutoff = 1.0;
        float t_e = 0.005f;

        // The filtered derivative of the signal.
        for (int i =0; i < joint_size_;i++){

            float x = static_cast<float>(motor_tor_[i]);
            float a_d = smoothing_factor(t_e, d_cutoff);
            float dx = static_cast<float>((x - euro_x_prev_[i]) / t_e);
            float dx_hat = static_cast<float>(exponential_smoothing(a_d, dx, static_cast<float>(euro_dx_prev_[i])));

            // The filtered signal.
            float cutoff = min_cutoff +  beta * abs(dx_hat);
            float aa = smoothing_factor(t_e, cutoff);
            float x_hat = static_cast<float>(exponential_smoothing(aa, x, static_cast<float>(euro_x_prev_[i])));

            euro_x_prev_[i] = x_hat;
            euro_dx_prev_[i] = dx_hat;
            filtered_motor_tor_euro_[i] = x_hat;
        }
        // Memorize the previous values.
        filtered_joint_tor_euro_ = robotics_->convertMotorJoint(filtered_motor_tor_euro_, ConvertType::kJoint, true);
        // std::cout << "filtered_motor_tor_: "<<filtered_motor_tor_.transpose() << std::endl;
}

void SoleArmPlanner::updatePlannerState(const RobotState& ro_state){
    last_joint_pos_ = joint_pos_;
    last_joint_vel_ = joint_vel_;
    last_motor_vel_ = motor_vel_;

    if (!is_simulation_){
        for (int i =0; i<joint_size_; i++){
            motor_pos_[i] = (double)ro_state.MotorPos[arm_i_][i];
            motor_vel_[i] = (double)ro_state.MotorVel[arm_i_][i];
            if (ro_state.enable_joint_torque_sensor == true){
                joint_tor_[i] = (double)ro_state.JointTorqueSensor[arm_i_][i];
                motor_tor_ = robotics_->convertMotorJoint(joint_tor_, ConvertType::kMotor, true);
            }else{
                motor_tor_[i] = (double)ro_state.MotorTor[arm_i_][i];
            }
        }
    }

    //Calculate pose, Jpos, Jvel and Jtor with motor data
    if (!is_simulation_){
        joint_pos_ = robotics_->convertMotorJoint(motor_pos_, ConvertType::kJoint, false);
        joint_vel_ = robotics_->convertMotorJoint(motor_vel_, ConvertType::kJoint, false);
        joint_tor_ = robotics_->convertMotorJoint(motor_tor_, ConvertType::kJoint, true);
    }else{
        switch (actuator_mode_){
            case ControlType::kPosition:
                motor_pos_ = motor_cmd_final_;
                joint_pos_ = robotics_->convertMotorJoint(motor_pos_, ConvertType::kJoint, false);
                joint_vel_ = (joint_pos_ - last_joint_pos_) / dt_;
                break;
            case ControlType::kVelocity:
                motor_vel_ = motor_cmd_final_;
                motor_pos_ = motor_pos_+motor_vel_*dt_;
                joint_vel_ = robotics_->convertMotorJoint(motor_vel_, ConvertType::kJoint, false);
                joint_pos_ = robotics_->convertMotorJoint(motor_pos_, ConvertType::kJoint, false);
                break;
            default:
                motor_tor_ = motor_cmd_final_;
                joint_tor_ = robotics_->convertMotorJoint(motor_tor_, ConvertType::kJoint, true);
                break;
        }
    }
    computeFK(joint_pos_);

    //Estimate Acceleration
    joint_acc_ = (joint_vel_ - last_joint_vel_) / dt_;
    motor_acc_ = (motor_vel_ - last_motor_vel_) / dt_;
    
    //Save to recording
    if (enable_recording_ && motion_type_ != MotionControl::kPlayback){
        if (!max_record_size_reached_){
            record_trajectory_.push_back(joint_pos_); 
            if (record_trajectory_.size() == max_record_trajectory_size_){
                max_record_size_reached_ = true;
                std::cerr << "Recording stopped. Maximum buffer limit of " 
                          << kMaxRecordDurationMinute_ << " minutes (" 
                          << max_record_trajectory_size_ << " points) reached." << std::endl;
            }
        }
    }

    //Update PlannerState 
    for (int i =0; i<joint_size_; i++){
        planner_state_.MotorPos[arm_i_][i] = static_cast<float>(motor_pos_[i]);
        planner_state_.MotorVel[arm_i_][i] = static_cast<float>(motor_vel_[i]);
        planner_state_.MotorTor[arm_i_][i] = static_cast<float>(motor_tor_[i]);
        planner_state_.JointPos[arm_i_][i] = static_cast<float>(joint_pos_[i]);
        planner_state_.JointVel[arm_i_][i] = static_cast<float>(joint_vel_[i]);
        planner_state_.JointTor[arm_i_][i] = static_cast<float>(joint_tor_[i]);
        VectorXd row = joint_pose_.row(i);
        for (int j=0; j<7; j++){
            planner_state_.JointPose[arm_i_][i][j] = static_cast<float>(row[j]);
        }
        
        planner_state_.joint_acceleration[arm_i_][i] = static_cast<float>(joint_acc_[i]);
        planner_state_.FilteredMotorTor[arm_i_][i] = static_cast<float>(filtered_motor_tor_[i]);
        planner_state_.FilteredMotorTorKalman[arm_i_][i] = static_cast<float>(filtered_motor_tor_kalman_[i]);
        planner_state_.FilteredMotorTorEuro[arm_i_][i] = static_cast<float>(filtered_motor_tor_euro_[i]);
    }
    for (int j=0; j<7; j++){
        planner_state_.EEPose[arm_i_][j] = static_cast<float>(ee_pose_[j]);
        planner_state_.ToolPose[arm_i_][j] = static_cast<float>(tool_pose_[j]);
    }

    // Current joint position may be out of joint limits if under Vel/Tor control.
    if (actuator_mode_ != ControlType::kPosition && reachJointLimit(joint_pos_, reached_limit_ji_)){
        for (int i=0; i<joint_size_; i++){
            int side = reached_limit_ji_[i];
            if (side > -1){
                planner_state_.arm_joint_diagnostic_flags[arm_i_][i] |= (actuator_mode_ == ControlType::kVelocity)? DiagnosticFlags::kBoundaryVelClamp : DiagnosticFlags::kBoundaryJointImpedance;
                if (!is_reached_limits_) {
                    std::cout   << "Arm " << arm_i_ << " Joint " << i << " reached position limit."
                                <<" Cur pos: " << joint_pos_[i]*180/M_PI << " deg"
                                << ", " << ((side == 0)? "Lower" : "Upper") << " limit: " << joint_pos_limits_(side,i)*180/M_PI << " deg." << std::endl;
                }
            }
        }

        if (!is_reached_limits_){
            is_reached_limits_ = true;
            limited_joint_vel_ = joint_vel_;
            limited_joint_tor_ = joint_tor_;
        }
    }else{
        is_reached_limits_ = false;
    }
}

// Saturation on joint command
VectorXd SoleArmPlanner::saturate(const VectorXd& v, bool is_top){
    Matrix2Xf* limit_ptr; 
    ControlType control_type = (is_top)? target_type_ : actuator_mode_;
    uint32_t warning_flag = DiagnosticFlags::kNone;
    switch (control_type){
        case ControlType::kPosition: //Saturation on position
            limit_ptr = &joint_pos_limits_;
            warning_flag = (is_top)? DiagnosticFlags::kTargetPosSaturation : DiagnosticFlags::kActuatorPosSaturation;
            break;
        case ControlType::kVelocity:
            limit_ptr = &joint_vel_limits_;
            warning_flag = (is_top)? DiagnosticFlags::kTargetVelSaturation : DiagnosticFlags::kActuatorVelSaturation;
            break;
        default:
            limit_ptr = &joint_tor_limits_;
            warning_flag = (is_top)? DiagnosticFlags::kTargetTorSaturation : DiagnosticFlags::kActuatorTorSaturation;
            break;
    }

    VectorXd sat_v(v.size());
    for (int i=0; i<v.size(); i++){
        double a = limit_ptr->row(0)[i];
        double b = limit_ptr->row(1)[i];
        sat_v[i] = std::max(a, std::min(b, v[i]));
        if (sat_v[i] != v[i]){
            planner_state_.arm_joint_diagnostic_flags[arm_i_][i] |= warning_flag;
        }
    }

    return sat_v;
}

VectorXd SoleArmPlanner::limitJointJump(const VectorXd& v, bool is_top, float k){
    VectorXd sat_v = v;
    VectorXf* limit_ptr;
    VectorXd* prev_state_ptr;
    uint32_t warning_flag;
    ControlType control_type = (is_top)? target_type_ : actuator_mode_;
    if (control_type == ControlType::kPosition){
        warning_flag = DiagnosticFlags::kActuatorPosJumpSaturation;
        limit_ptr = &joint_jump_pos_limits_;
        prev_state_ptr = &joint_pos_;
    }else if (control_type == ControlType::kVelocity){
        warning_flag = DiagnosticFlags::kActuatorVelJumpSaturation;
        limit_ptr = &joint_jump_vel_limits_;
        prev_state_ptr = &joint_vel_;
    }else if (control_type == ControlType::kTorque){
        warning_flag = DiagnosticFlags::kActuatorTorJumpSaturation;
        limit_ptr = &joint_jump_tor_limits_;
        prev_state_ptr = &joint_tor_;
    }else{
        return v;
    }

    const VectorXf& limit = *limit_ptr;
    const VectorXd& prev_state = *prev_state_ptr;
    const VectorXd& prev_cmd = (joint_jump_last_actuator_mode_ == actuator_mode_ && !is_top)? joint_jump_last_joint_cmd_ : prev_state;

    for (int i=0; i<v.size(); i++){
        float final_limit = limit[i];
        if (std::abs(sat_v[i] - prev_state[i]) > final_limit){
            sat_v[i] = (sat_v[i] > prev_state[i])? prev_state[i] + final_limit : prev_state[i] - final_limit;
            planner_state_.arm_joint_diagnostic_flags[arm_i_][i] |= warning_flag;
        }

        if (std::abs(sat_v[i] - prev_cmd[i]) > final_limit){
            sat_v[i] = (sat_v[i] > prev_cmd[i])? prev_cmd[i] + final_limit : prev_cmd[i] - final_limit;
            planner_state_.arm_joint_diagnostic_flags[arm_i_][i] |= warning_flag;
        }
    }
    if (!is_top){
        joint_jump_last_actuator_mode_ = actuator_mode_;
        joint_jump_last_joint_cmd_ = sat_v;
    }
    return sat_v;
}


VectorXd SoleArmPlanner::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau) const{
    return robotics_->convertMotorJoint(X, convert_to, is_tau);
}

VectorXd SoleArmPlanner::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau, int arm_i) const{
    return convertMotorJoint(X, convert_to, is_tau);
}

// calculate motors' command during motor control mode
void SoleArmPlanner::setMotorVal(const VectorXd& tar){
    // VectorXd x0 = motor_pos_, v0 = motor_vel_, tor0 = motor_tor_, target;
    // target = tar;
    // // if (actuator_mode_ == ControlType::kPosition){
    // //     target = x0;
    // // }else if (actuator_mode_ == ControlType::kVelocity){
    // //     target = v0;
    // // }else if (actuator_mode_ == ControlType::kTorque){
    // //     target = tor0;
    // // }
    // // for (MotorDevice* m: motors_){
    // //     int i = m->id;
    // //     target[i] = m->target;
    // // }

    // sliced_motor_tar_ = target;
    // motor_tar_ = target;
    // sat_motor_cmd_ = target;
    // if (control_algorithm_ == ControlAlgorithm::kJointAdmittance || control_algorithm_ == ControlAlgorithm::kTaskAdmittance){
    //     VectorXd joint_vel_cmd = VectorXd::Zero(joint_size_);
    //     double err, sgn, adjusted_err = 0;
    //     for (int i=0; i < joint_tor_.size(); i++){
    //         err = abs(joint_tor_[i] + joint_tor_gravity_[i]) - admittance_threshold_;
    //         if (err > 0){
    //             sgn = (joint_tor_[i] < 0)? -1.0 : 1.0;
    //             adjusted_err = std::max(0.0, std::min(err, (double)admittance_force_upper_bound_));
    //             adjusted_err *= M_PI/admittance_force_upper_bound_;
    //             joint_vel_cmd[i] = admittance_max_speed_*sgn*(1-std::cos(adjusted_err));
    //         }
    //     }
    //     motor_cmd_= robotics_->convertMotorJoint(joint_vel_cmd, ConvertType::kMotor, false);
    // }else{
    //     sliced_motor_tar_ = (control_algorithm_ == ControlAlgorithm::kNone)? getSlicedMotorVal(target,x0,v0,tor0) : VectorXd::Zero(joint_size_);
    //     motor_tar_ = sliced_motor_tar_;
    //     motor_cmd_ = PIDControl(sliced_motor_tar_,x0,v0,tor0);
    // }

    // bool isTor = (actuator_mode_ == ControlType::kTorque);
    // sat_joint_cmd_ = saturate(robotics_->convertMotorJoint(motor_cmd_, ConvertType::kJoint, isTor), false); // Saturation is on Joint level
    // sat_motor_cmd_ = robotics_->convertMotorJoint(sat_joint_cmd_, ConvertType::kMotor, isTor);
    // if (is_simulation_)
    //     updateMotorInSimulation(sat_motor_cmd_);
}

/** Get sliced motor value
 * 
 * \param target    motor target value
 * \param x0        motor current position
 * \param v0        motor current velocity
 * \param tor0      motor current torque
 */
VectorXd SoleArmPlanner::getSlicedMotorVal(VectorXd target, VectorXd x0, VectorXd v0, VectorXd tor0){
    if (target_type_ == ControlType::kPosition){
        return motor_inter_.getNextPos(x0, target);
    }else if (target_type_ == ControlType::kVelocity){
        std::cout << "Need Debug! SoleArmPlanner::getSlicedMotorVal kVelocity -> Why x0 is not used?" << std::endl;
        return motor_inter_.getNextPos(VectorXd::Zero(joint_size_), target);
    }else if (target_type_ == ControlType::kTorque){
        std::cout << "Need Debug! SoleArmPlanner::getSlicedMotorVal kTorque -> Why x0 is not used?" << std::endl;
        return motor_inter_.getNextPos(VectorXd::Zero(joint_size_), target);
    }
    return VectorXd::Zero(joint_size_);
}

VectorXd SoleArmPlanner::guardJointPositionLimits(VectorXd val){
    if (actuator_mode_ == ControlType::kVelocity && is_reached_limits_){
        if (robotics_->anySameSign(val, limited_joint_vel_, reached_limit_ji_))  //If exceeded joint position and current velocity is in the same direction
            return VectorXd::Zero(joint_size_);  //Stop immediately when reaching limit under velocity control
    }else if (actuator_mode_ == ControlType::kTorque && control_algorithm_ != ControlAlgorithm::kJointImpedance && is_reached_limits_){
        if (robotics_->anySameSign(val, limited_joint_tor_, reached_limit_ji_) 
            || control_algorithm_ == ControlAlgorithm::kGravity || control_algorithm_ == ControlAlgorithm::kGravityWithBoundary){
            VectorXd target = clampJointPos(joint_pos_);
            VectorXd joint_tor_impedance = robotics_->computeJointImpedance(joint_pos_, joint_vel_, joint_tor_gravity_, target, dt_);
            VectorXd joint_tor_impedance_without_gravity = joint_tor_impedance - joint_tor_gravity_;
            return val + joint_tor_impedance_without_gravity;
        }
    }
    return val;
}

/** Get motor command
 * 
 * \param val       motor sliced target value
 * \param x0        motor current position
 * \param v0        motor current velocity
 * \param tor0      motor current torque
 */
VectorXd SoleArmPlanner::PIDControl(VectorXd val, VectorXd x0, VectorXd v0, VectorXd tor0){
    VectorXd cmd = VectorXd::Zero(joint_size_);
    for (int i =0; i<joint_size_; i++){
        double tar = val[i], x = x0[i], v = v0[i], t = tor0[i];
        if (target_type_ == ControlType::kPosition && actuator_mode_ == ControlType::kPosition){
            cmd[i] = tar;
        }else if (target_type_ == ControlType::kPosition && actuator_mode_ == ControlType::kVelocity){
            cmd[i] = pos_vel_kp_[i]*(tar-x) + motor_vel_offset_[i];
        }else if (target_type_ == ControlType::kPosition && actuator_mode_ == ControlType::kTorque){
            cmd[i] = pos_tor_kp_[i]*(tar-x) + pos_tor_kd_[i]*(0-v) + motor_tor_offset_[i];
        }else if (target_type_ == ControlType::kVelocity && actuator_mode_ == ControlType::kVelocity){
            cmd[i] = tar + motor_vel_offset_[i];
        }else if (target_type_ == ControlType::kVelocity && actuator_mode_ == ControlType::kTorque){
            cmd[i] = vel_tor_kp_[i]*(tar-v) + vel_tor_kd_[i]*(0-t) + motor_tor_offset_[i];
        }else if (target_type_ == ControlType::kTorque && actuator_mode_ == ControlType::kTorque){
            cmd[i] = tar + motor_tor_offset_[i];
        }
    }
    return cmd;
}

// Update motor current state during simulation
void SoleArmPlanner::updateMotorInSimulation(VectorXd val){
    if (actuator_mode_ == ControlType::kPosition){
        motor_pos_ = val;
        motor_vel_ = VectorXd::Zero(joint_size_);
        motor_tor_ = VectorXd::Zero(joint_size_);
    }else if (actuator_mode_ == ControlType::kVelocity){
        motor_vel_ = val;
        motor_pos_ += motor_vel_*dt_;
        motor_tor_ = VectorXd::Zero(joint_size_);
    }else{
        motor_tor_ = val;
        motor_pos_ = VectorXd::Zero(joint_size_);
        motor_vel_ = VectorXd::Zero(joint_size_);
    }
}

/** Perform interpolation of target before calling getPIDCommand().
 * 
 * \param tar target in rad
 */
void SoleArmPlanner::setJointVal(const VectorXd& tar){
    VectorXd sat_tar = saturate(tar, true);
    if (arm_target_mode_ == PanelTargetMode::kSinglePoint || arm_target_mode_ == PanelTargetMode::kRawSinglePoint){
        if (target_type_ == ControlType::kPosition){
            sliced_joint_tar_ = joint_inter_.getNextPos(joint_pos_, sat_tar);
            // sliced_joint_tar_ = joint_inter_.getNextPos(joint_pos_, sat_tar, joint_vel_, VectorXd::Zero(joint_size_), joint_acc_);
        }else if (target_type_ == ControlType::kVelocity){
            sliced_joint_tar_ = joint_inter_.getNextPos(joint_vel_, sat_tar);
        }else{
            sliced_joint_tar_ = joint_inter_.getNextPos(joint_tor_, sat_tar);
        }
    }else if (arm_target_mode_ == PanelTargetMode::kWaypoint){
        if (target_type_ == ControlType::kPosition){
            sliced_joint_tar_ = joint_inter_.getNextPos(joint_pos_, arm_waypoint_cmd_, arm_waypoint_time_, arm_waypoint_leading_joint_max_vel_);
        }else if (target_type_ == ControlType::kVelocity){
            sliced_joint_tar_ = joint_inter_.getNextPos(joint_vel_, arm_waypoint_cmd_, arm_waypoint_time_, arm_waypoint_leading_joint_max_vel_);
        }else{
            sliced_joint_tar_ = joint_inter_.getNextPos(joint_tor_, arm_waypoint_cmd_, arm_waypoint_time_, arm_waypoint_leading_joint_max_vel_);
        }
    }

    if (motion_type_ == MotionControl::kJoint || motion_type_ == MotionControl::kControlAlgorithm){
        joint_inter_.getPositionTarget(target_before_interpolation_);
    }else if (motion_type_ == MotionControl::kTask){
        if (arm_target_mode_ == PanelTargetMode::kWaypoint){
            int target_ind = joint_inter_.getPositionTargetIndex();
            target_before_interpolation_ = last_task_waypoint_cmd_[target_ind];
        }else{
            target_before_interpolation_ = sliced_task_tar_;
        }
    }else{
        throw std::runtime_error("motion_type_ '" + std::to_string(static_cast<int>(motion_type_)) + "' is not supported for setJointVal");
    }

    planner_state_.system_diagnostic_flags |= joint_inter_.getWarningFlags();
    
    //Advoid interpolation from current torque to 0 when changed to force control.
    if (motion_type_ == MotionControl::kControlAlgorithm){
        if (control_algorithm_ != ControlAlgorithm::kNone && control_algorithm_ != ControlAlgorithm::kNullAdmittance){
            sliced_joint_tar_ = VectorXd::Zero(joint_size_);
        }else if (control_algorithm_ == ControlAlgorithm::kNullAdmittance){
            sliced_joint_tar_ = joint_pos_;
        }
    }
    getPIDCommand(sliced_joint_tar_);
}

/** Calculate forward kinematics and update robot pose, motor position.
 * 
 * \param tar target in rad
 */
VectorXd SoleArmPlanner::getPIDCommand(const VectorXd& tar){
    bool isTor = (target_type_ == ControlType::kTorque);
    motor_tar_ = robotics_->convertMotorJoint(tar, ConvertType::kMotor, isTor);  //from joint cmd to motor cmd
    pid_cmd_ = PIDControl(motor_tar_,motor_pos_,motor_vel_,motor_tor_);    //convert base on control_type
    return pid_cmd_;
}

void SoleArmPlanner::updateRobotCmdTarget(VectorXd val){
    for (int i=0; i<joint_size_; i++){
        // planner_command_.Target[arm_i_][i] = (actuator_mode_ == ControlType::kTorque)? val[i] : val[i]*180/M_PI;
        planner_command_.Target[arm_i_][i] = static_cast<float>(val[i]);
    }
}


void SoleArmPlanner::computeFK(const VectorXd& joint_pos){
    joint_pose_ = robotics_->FK(joint_pos, &ee_pose_, &tool_pose_);
}

/** Calculate forward kinematics and update robot pose, motor position.
 * 
 * \param joint_pos Joint position in rad
 */
void SoleArmPlanner::computeFK(const VectorXd& joint_pos, Vector7d& ee_pose, Vector7d& tool_pose) const {
    robotics_->FK(joint_pos, &ee_pose, &tool_pose);
}

bool SoleArmPlanner::computeQuIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const {
    return robotics_->QuIK(R, P, res);
}

VectorXd SoleArmPlanner::clampJointPos(const VectorXd&v){
    VectorXd sat_v = v;
    for (int i=0; i<joint_size_; i++){
        if (reached_limit_ji_[i] == -1) continue;
        int limit_i = reached_limit_ji_[i];
        sat_v[i] = joint_pos_limits_(limit_i,i);
    }
    return sat_v;
}

/** Check if all joint pos are within limits.
 * 
 * \param joint_pos Joint position in rad
 */
bool SoleArmPlanner::reachJointLimit(const VectorXd& joint_pos, VectorXi& reached_limit_ji_){
    reached_limit_ji_ = -1*VectorXi::Ones(joint_size_);
    bool reached = false;
    for (int i=0; i<joint_size_; i++){
        if (joint_pos[i] < joint_pos_limits_(0,i)){
            reached_limit_ji_[i] = 0;
            reached = true;
        }else if (joint_pos[i] > joint_pos_limits_(1,i)){
            reached_limit_ji_[i] = 1;
            reached = true;
        }
    }
    return reached;
}

/** Calculate inverse kinematics and update robot pose, motor position.
 * 
 * \param tar End effector pose
 */
void SoleArmPlanner::setTaskLineVal(const Vector6d& tar, LineMoveStrategy strategy){
    if (target_type_ != ControlType::kPosition){
        throw std::runtime_error("Task line control is not supported for non-position control");
    }

    long t0=get_time_now();
    Vector6d next_cmd;
    if (arm_target_mode_ == PanelTargetMode::kSinglePoint){
        next_cmd = task_inter_.getNextPos(sliced_task_tar_, tar);
    }else if (arm_target_mode_ == PanelTargetMode::kWaypoint){
        last_task_waypoint_cmd_ = arm_waypoint_cmd_;
        next_cmd = task_inter_.getNextPos(sliced_task_tar_, arm_waypoint_cmd_, arm_waypoint_time_, arm_waypoint_leading_joint_max_vel_);
    }
    task_inter_.getPositionTarget(target_before_interpolation_);
    planner_state_.system_diagnostic_flags |= task_inter_.getWarningFlags();
    VectorXd q = last_task_solved_joint_cmd_;
    bool is_new_target = task_inter_.getInterpolatorIndex() == 1;
    
    //Handle for different strategies
    bool skip_ik = task_inter_.isInterrupted();
    if (!skip_ik && is_new_target && strategy == LineMoveStrategy::kRejectEntirely){ 
        //Validate if subpoints are reachable
        float t = 0;
        float dt = dt_*10;
        float interpolation_t = task_inter_.getInterpolationT();
        VectorXd prev_q = q;
        VectorXd initial_sliced_task_tar_ = sliced_task_tar_;
        long t001 = get_time_now();
        while ( t < interpolation_t ){
            t += dt;
            VectorXd pos_at_time = task_inter_.getPosAtTime(t);
            bool success = setTaskValInternal(pos_at_time, prev_q);
            if (!success){
                skip_ik = true;
                task_inter_.interrupt();
                planner_state_.plan_result = PlanResult::kLinearPathFailed;
                break;
            }
            prev_q = pos_at_time;
        }
        long t002 = get_time_now();
        std::cout <<"LineMoveStrategy::RejectEntirely varification cost: " << t002 - t001 << "us" << ", t: " << t << ", T: " << interpolation_t <<", dt: " << dt << ", skip_ik: " << skip_ik << std::endl;
        
        //Reset params that may be changed inside setTaskValInternal 
        sliced_task_tar_ = initial_sliced_task_tar_;
        last_task_solved_joint_cmd_ = q;
    }

    //IK only if previous sliced target is reachable or received new valid target
    if (planner_state_.plan_result == PlanResult::kSuccess || (is_new_target && !skip_ik)){
        bool success = setTaskValInternal(next_cmd, q);
        planner_state_.plan_result = (success)? PlanResult::kSuccess : PlanResult::kLinearPathFailed;
        if (!success){
            task_inter_.interrupt();
        }
    }

    long t1 = get_time_now();
    getPIDCommand(q);
    planner_state_.IK_cost = t1 - t0;
}

bool SoleArmPlanner::setTaskValInternal(const Vector6d& tar, VectorXd& q){
    bool success = true;
    if (!sliced_task_tar_.isApprox(tar, 1e-7)){ //1e-5 is the threshold for the difference of the task target
        VectorXd q0 = last_task_solved_joint_cmd_;
        Vector3d P = tar.head(3) + task_base_end_pose_.head(3);

        Vector3d rpy_increment = tar.tail(3);
        Matrix3d R1 = robotics_->toR(rpy_increment);
        Matrix3d R0 = task_base_R_;
        Matrix3d R = (task_orient_type_ == OrientControl::kEnd)? R0*R1 : R1*R0;
        
        // bool success = robotics_->geometricIK(R, P, q0);
        success = robotics_->IK(R, P, q0);
        // success = robotics_->QuIK(R, P, q0);
        if (success){
            sliced_task_tar_ = tar;
            // task_base_R_ = R;
            last_task_solved_joint_cmd_ = q0;
        }
        q = q0;
    }
    return success;
}

void SoleArmPlanner::setTaskVal(const Vector6d& tar){
    if (target_type_ != ControlType::kPosition){
        throw std::runtime_error("Task control is not supported for non-position control");
    }

    long t0 = get_time_now();
    VectorXd q = last_task_solved_joint_cmd_;
    bool success = true;
    if (arm_target_mode_ == PanelTargetMode::kSinglePoint){
        success = setTaskValInternal(tar, q);
        planner_state_.plan_result = (success)? PlanResult::kSuccess : PlanResult::kPoseNotReachable;
    }else if (arm_target_mode_ == PanelTargetMode::kWaypoint){
        //Check if waypoint is different from last command
        if (last_task_waypoint_cmd_.size() <= 0){
            last_task_waypoint_res_.clear();
            last_task_waypoint_cmd_.clear();
            for (int i = 0; i < arm_waypoint_cmd_.size(); i++){
                Vector6d waypoint_tar = arm_waypoint_cmd_[i].head(6);
                success &= setTaskValInternal(waypoint_tar, q);
                last_task_waypoint_res_.push_back(q);
                last_task_waypoint_cmd_.push_back(waypoint_tar);
                q = last_task_solved_joint_cmd_;
            }
            planner_state_.plan_result = (success)? PlanResult::kSuccess : PlanResult::kPoseNotReachable;
        }
        arm_waypoint_cmd_ = last_task_waypoint_res_;
    }

    long t1 = get_time_now();
    if (planner_state_.plan_result == PlanResult::kSuccess){
        setJointVal(q);
    }
    planner_state_.IK_cost = t1 - t0;
}

void SoleArmPlanner::setNullVal(const VectorXd& tar){
    //Must run in Position/Vel
    long t0, t1;
    t0 = get_time_now();
    std::string method = "geometric";
    if (method == "geometric"){
        VectorXd sliced = joint_inter_.getNextPos(sliced_null_tar_, tar);
        bool success = robotics_->geometricNullSpaceControl(sliced[0], null_base_R_, null_base_P_, null_base_P4_, joint_pos_);
        t1 = get_time_now();
        getPIDCommand(joint_pos_);
        if (success){
            sliced_null_tar_ << sliced;
        }
        sliced_joint_tar_ = sliced_null_tar_;
    }else if (method == "jacobian"){ 
        // **Tested on simulation but not robot**
        // Jacobian Null Space control can be viewed as multi tasks control. 
        // First priority is end-effector stay same pose. [inv_J*dot_x]
        // Second priority is to perform null space control. [(I - inv_J*J)*dot_q0]
        // dot_q = inv_J*dot_x + (I - inv_J*J)*dot_q0, where dot_q0 is the second task joint vel.

        //Second task: null space control
        VectorXd sliced = joint_inter_.getNextPos(sliced_null_tar_, tar);
        double j3_vel = 2*(sliced[0] - sliced_null_tar_[0])/dt_; 
        VectorXd dot_q0 = joint_vel_;
        dot_q0[2] = j3_vel;
        VectorXd joint_vel_offset = robotics_->jacobianNullSpaceControl(joint_pos_, dot_q0);
        motor_vel_offset_ = robotics_->convertMotorJoint(joint_vel_offset, ConvertType::kMotor, false);
        t1 = get_time_now();
        
        //First task: Stay at current pose
        getPIDCommand(joint_pos_);
        sliced_null_tar_ << sliced;
        sliced_joint_tar_ = sliced_null_tar_;
    }

    planner_state_.Null_cost = t1 - t0;
}

void SoleArmPlanner::setPlaybackVal(){
    if (target_type_ != ControlType::kPosition){
        throw std::runtime_error("Playback control require target type set to ControlType::kPosition");
    }

    if (record_trajectory_.size() <= 0){
        return;
    }

    if (playback_switched_cmd_ && playback_cmd_ != PlaybackState::kStop ){ //Start new playback
        playback_index_ = 0;
        playback_trajectory_.clear();
        joint_inter_.reset();
        //Interpolation from current position to the start of playback trajectory
        VectorXd cur_pos = joint_pos_;
        do {
            VectorXd sliced_pos = joint_inter_.getNextPos(cur_pos, record_trajectory_[0]);
            playback_trajectory_.push_back(sliced_pos);
            cur_pos = sliced_pos;
        } while (joint_inter_.getState() != InterpolationMotionPhase::kFinished);

        //Append other points in record_trajectory
        if (playback_cmd_ == PlaybackState::kStart){
            playback_trajectory_.insert(playback_trajectory_.end(), std::next(record_trajectory_.begin()), record_trajectory_.end());
        }
        playback_switched_cmd_ = false;
    }
    
    if (playback_index_ < playback_trajectory_.size() && playback_cmd_ != PlaybackState::kStop){
        sliced_joint_tar_ = playback_trajectory_[playback_index_];
        if (playback_index_ < playback_trajectory_.size() - 1) playback_index_++;
    }
    getPIDCommand(sliced_joint_tar_);
}

void SoleArmPlanner::resetControlMem(MotionControl t){
    int last_joint_id = joint_size_-1;
    Vector7d ee_pose;
    robotics_->FK(joint_pos_, &ee_pose);
    task_base_end_pose_ = ee_pose;
    Vector4d quat0 = task_base_end_pose_.tail(4);
    if (t == MotionControl::kTask || t == MotionControl::kTaskLine){
        sliced_task_tar_ = Vector6d::Zero();
        last_task_solved_joint_cmd_ = joint_pos_;
        task_base_joint_pos_ = joint_pos_;
        task_base_R_ = robotics_->toR(quat0);
        task_base_rpy_ = robotics_->toEulerAngle(task_base_R_);
        last_task_waypoint_cmd_.clear();
        last_task_waypoint_res_.clear();
    }else if (t == MotionControl::kNull){
        sliced_null_tar_ = VectorXd::Zero(joint_size_);
        null_base_P_ = task_base_end_pose_.head(3);
        null_base_R_ = robotics_->toR(quat0);
        null_base_P4_ = joint_pose_.row(3).head(3);
    }
}

void SoleArmPlanner::setGravityWithBoundaryTarget(){
    // Vector3d task_pos = joint_pose_.row(6).head(3);
    // gravity_with_boundary_target_pos_ = task_pos;
}

void SoleArmPlanner::setImpedanceTarget(){
    // Vector3d task_pos = joint_pose_.row(6).head(3);
    // Eigen::Quaterniond task_ori;
    // task_ori.w() = joint_pose_.coeff(6,3);
    // task_ori.vec() = joint_pose_.row(6).tail(3);
    // VectorXd joint_pos = joint_pos_;

    // task_impedance_target_pos_ = task_pos;
    // task_impedance_target_ori_ = task_ori;
    // joint_impedance_target_ = joint_pos;
}

// Compute torque using inverse dynamics
void SoleArmPlanner::computeID(){
    motor_vel_offset_ = VectorXd::Zero(joint_size_);
    motor_tor_offset_ = VectorXd::Zero(joint_size_);
    joint_tor_gravity_ = robotics_->computeGravity(joint_pos_, joint_vel_);
    motor_tor_gravity_ = robotics_->convertMotorJoint(joint_tor_gravity_, ConvertType::kMotor, true);
    for (int i =0; i<joint_size_; i++){
        planner_state_.joint_gravity[arm_i_][i] = static_cast<float>(joint_tor_gravity_[i]);
        planner_state_.gravity[arm_i_][i] = static_cast<float>(motor_tor_gravity_[i]);
    }
    
    if (motion_type_ != MotionControl::kControlAlgorithm) return;
    VectorXd joint_val;
    switch (control_algorithm_){
        case ControlAlgorithm::kGravity:
            motor_tor_offset_ = motor_tor_gravity_;
            break;
        // case ControlAlgorithm::kGravityWithBoundary:
        //     joint_val = robotics_->computeGravityWithBoundary(joint_pos_, joint_vel_, joint_tor_gravity_, gravity_with_boundary_target_pos_);
        //     motor_tor_offset_ = robotics_->convertMotorJoint(joint_val, ConvertType::kMotor, true);
        //     break;
        // case ControlAlgorithm::kJointAdmittance:
        //     joint_val = robotics_->computeJointAdmittance(joint_pos_, joint_vel_, filtered_joint_tor_euro_, joint_tor_gravity_, admittance_last_cmd_, dt_);
        //     admittance_last_cmd_ = joint_val;
        //     motor_vel_offset_ = robotics_->convertMotorJoint(joint_val, ConvertType::kMotor, false);
        //     break;
        // case ControlAlgorithm::kTaskAdmittance:
        //     break;
        // case ControlAlgorithm::kJointImpedance:
        //     joint_val = robotics_->computeJointImpedance(joint_pos_, joint_vel_, joint_tor_gravity_, joint_impedance_target_, dt_);
        //     motor_tor_offset_ = robotics_->convertMotorJoint(joint_val, ConvertType::kMotor, true);
        //     for (int i =0; i<joint_size_; i++){
        //         planner_state_.joint_impedance[arm_i_][i] = joint_val[i];
        //         planner_state_.impedance[arm_i_][i] = motor_tor_offset_[i];
        //     }
        //     // }else if (task_admittance_ && motion_type_ == MotionControl::kJoint){
        //     //     // joint_vel_offset = robotics_->computeTaskAdmittance(joint_pos_, joint_tor_);
        //     //     // motor_vel_offset_ = robotics_->convertMotorJoint(joint_vel_offset, ConvertType::kMotor, false);
        //     break;
        // case ControlAlgorithm::kTaskImpedance:
        //     joint_val = robotics_->computeTaskImpedance(joint_pos_, joint_vel_, joint_acc_, joint_tor_gravity_, task_impedance_target_pos_, task_impedance_target_ori_, dt_);
        //     motor_tor_offset_ = robotics_->convertMotorJoint(joint_val, ConvertType::kMotor, true);
        //     for (int i =0; i<joint_size_; i++){
        //         planner_state_.joint_impedance[arm_i_][i] = joint_val[i];
        //         planner_state_.impedance[arm_i_][i] = motor_tor_offset_[i];
        //     }
        //     break;
        // case ControlAlgorithm::kNullAdmittance:
        // //         joint_val = robotics_->computeNullAdmittance(joint_pos_, filtered_joint_tor_euro_, null_base_R_, null_base_P_, null_base_P4_, sliced_null_tar_[0], dt_);
        // //         motor_vel_offset_ = getPIDCommand(joint_val); 
        // //         admittance_last_cmd_ = joint_val;
        // //         sliced_joint_tar_ = sliced_null_tar_;
        // //         for (int i =0; i <7; i++){
        // //             planner_state_.null_joint_pos[arm_i_][i] = joint_val[i];
        //     break;
        default:
            break;
    }
}
