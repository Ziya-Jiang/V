#include "dual_v1_arm.h"

#include "src/file_utils.h"
#include "hardware_modules/eyou_dual_arm-multi_can/include/config_loader.h"
#include "hardware_modules/eyou_dual_arm-multi_can/include/callback_dispatcher.h"

DualV1Arm::DualV1Arm(const std::string& name, int arm_i): ArmBase(name, kDefaultJointSize_), kArmIndex_(arm_i){
    ConfigLoader config_loader_ptr = ConfigLoader(get_absolute_path(kConfigPath_));
    RobotConfig& robot_config_ptr = config_loader_ptr.getRobotConfig();
    arm_ptr_ = std::make_unique<EyouJoints>(kArmIndex_, kDefaultJointSize_, 
        robot_config_ptr.motors[kArmIndex_].can_channel, robot_config_ptr.motors[kArmIndex_].use_pdo,
        robot_config_ptr.motors[kArmIndex_].ids.data(),
        robot_config_ptr.motors[kArmIndex_].enabled.data(), robot_config_ptr.motors[kArmIndex_].gear_ratios.data(),
        robot_config_ptr.motors[kArmIndex_].sign.data(), robot_config_ptr.kinematics[kArmIndex_].initial_position.data());
    
    //Load parameters
    for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
        sign_[joint_i] = robot_config_ptr.motors[kArmIndex_].sign[joint_i];
        gear_ratio_[joint_i] = robot_config_ptr.motors[kArmIndex_].gear_ratios[joint_i];
        enable_motors_[joint_i] = (bool)robot_config_ptr.motors[kArmIndex_].enabled[joint_i];
        existing_motors_[joint_i] = (bool)robot_config_ptr.motors[kArmIndex_].enabled[joint_i];
    }
    can_channel_ = robot_config_ptr.motors[kArmIndex_].can_channel;
    pdo_auto_report_ms_ = robot_config_ptr.motors[kArmIndex_].pdo_auto_report_ms;
}

RetCode DualV1Arm::open(){
    // Initialize CAN communication and pdo callback dispatcher
    canbox_init({can_channel_});
    CallbackDispatcher::init();
    arm_ptr_->init_joints(pdo_auto_report_ms_);
    return RetCode::kSuccess;
};

RetCode DualV1Arm::close(){
    arm_ptr_->all_motor_stop();
    canbox_close({can_channel_});
    return RetCode::kSuccess;
};

RetCode DualV1Arm::setMode(ControlType mode){
    if (current_mode_ == mode) return RetCode::kSuccess; //Already in the desired mode
    harmonic_OperateMode op_mode = convertOperateMode(mode);
    bool success = (arm_ptr_->set_operate_mode(op_mode) == HARMONIC_SUCCESS);
    if (success) current_mode_ = mode;
    mode_changed_ = true;
    sleep_period(200); //Wait for mode change take effect
    return success ? RetCode::kSuccess : RetCode::kFailed;
};

RetCode DualV1Arm::setMode(int joint_i, ControlType mode){
    // Won't use and update current_mode_
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    harmonic_OperateMode op_mode = convertOperateMode(mode);
    bool success = (arm_ptr_->set_operate_mode(joint_i, op_mode) == HARMONIC_SUCCESS);
    mode_changed_ = true;
    sleep_period(200); //Wait for mode change take effect
    return success ? RetCode::kSuccess : RetCode::kFailed;
};

RetCode DualV1Arm::setCommand(int joint_i, float val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    if (current_mode_ == ControlType::kPosition) {
        arm_ptr_->set_mdhpos(joint_i, val * 180.0f/PI);
    } else if (current_mode_ == ControlType::kVelocity) {
        float vel_cmd = sign_[joint_i] * gear_ratio_[joint_i] * val * 180.0f/PI;
        arm_ptr_->set_velocity(joint_i, vel_cmd);
    }else if (current_mode_ == ControlType::kTorque) { 
        // arm_ptr_->set_torque(joint_i, val * sign_[joint_i]);
        arm_ptr_->set_torque(joint_i, 0.0);
    }else{
        std::cerr << "DualV1Arm::setCommand: Invalid control mode." << std::endl;
        return RetCode::kFailed;
    }
    return RetCode::kSuccess;
};

RetCode DualV1Arm::setCommandAndGetState(const std::vector<float>& set_val, std::vector<float>* pos, std::vector<float>* vel, std::vector<float>* tor, int set_delay_us, int read_delay_us){
    set_delay_us = mode_changed_ ? 200 + set_delay_us : set_delay_us; //If mode changed, use longer delay for first set
    mode_changed_ = false;
    return ArmBase::setCommandAndGetState(set_val, pos, vel, tor, set_delay_us, read_delay_us);
};

RetCode DualV1Arm::enableMotor(int joint_i, bool enable){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    if (enable_motors_[joint_i] == enable) return RetCode::kSuccess;
    enable_motors_[joint_i] = enable;
    
    bool success = true;
    if (enable){
        arm_ptr_->enable_motors_[joint_i] = true;
        if (current_mode_ != ControlType::kInvalid) success &= (setMode(joint_i, current_mode_) == RetCode::kSuccess);
    }else{
        arm_ptr_->motor_stop(joint_i, 2);
        arm_ptr_->enable_motors_[joint_i] = false;
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV1Arm::getPosition(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    int ret;
    float read_pos = arm_ptr_->get_mdhpos(joint_i, &ret) * PI / 180.0f;
    if (ret == HARMONIC_SUCCESS){
        *val = read_pos;
        return RetCode::kSuccess;
    }
    return RetCode::kFailed;
};

RetCode DualV1Arm::getVelocity(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    int ret;
    float read_vel = arm_ptr_->get_velocity(joint_i, &ret) * PI / 180.0f;
    if (ret == HARMONIC_SUCCESS){
        *val = read_vel * sign_[joint_i] / gear_ratio_[joint_i];
        return RetCode::kSuccess;
    }
    return RetCode::kFailed;
};

RetCode DualV1Arm::getTorque(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    int ret;
    float read_tor = arm_ptr_->get_current(joint_i, &ret);
    if (ret == HARMONIC_SUCCESS){
        *val = read_tor * sign_[joint_i];
        return RetCode::kSuccess;
    }
    return RetCode::kFailed;
};

harmonic_OperateMode DualV1Arm::convertOperateMode(ControlType mode){
    if (mode == ControlType::kPosition) {
        return harmonic_OperateMode::harmonic_OperateMode_CyclicSyncPosition;
    }else if (mode == ControlType::kVelocity) {
        return harmonic_OperateMode::harmonic_OperateMode_CyclicSyncVelocity;
    }else if (mode == ControlType::kTorque) { 
        std::cout << "Torque forced to 0 now.\n";
        return harmonic_OperateMode::harmonic_OperateMode_CyclicSyncTorque;
    }else {
        throw std::invalid_argument("DualV1Arm::convertOperateMode: Invalid control mode.");
    }
}

RetCode DualV1Arm::updateEncoderZeroOffset(int joint_i, float offset){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    // arm_ptr_->update_encoder_zero_offset(joint_i, offset);
    std::cout <<"DualV1Arm::updateEncoderZeroOffset need implement.\n";
    return RetCode::kSuccess;
}