#include "dual_v2_1_arm.h"

#include "src/file_utils.h"
#include "hardware_modules/realman_dual_arm/include/config_loader.h"
#include "hardware_modules/realman_dual_arm/include/can_fd.h"

DualV2_1Arm::DualV2_1Arm(const std::string& name, int arm_i): ArmBase(name, kDefaultJointSize_), kArmIndex_(arm_i){
    ConfigLoader config_loader(get_absolute_path(kConfigPath_));
    const auto& robot_config = config_loader.getRobots()[arm_i];
    arm_ptr_ = std::make_unique<RobotArms>(arm_i, robot_config);
    can_channel_ = robot_config.motors.can_channel;
    //Load parameters
    for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
        enable_motors_[joint_i] = (bool)robot_config.motors.enabled[joint_i];
        existing_motors_[joint_i] = (bool)robot_config.motors.enabled[joint_i];
    }
}

RetCode DualV2_1Arm::open(){
    canfd_init(can_channel_, 0);
    bool success = (arm_ptr_->initialize() == 0);
    //Must set to position mode first after disable/shutdown
    success &= (setMode(ControlType::kPosition) == RetCode::kSuccess); 
    if (!success) return RetCode::kFailed;
    success &= isAllBrakeOpened(5000000);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV2_1Arm::close(){
    setMode(ControlType::kPosition);
    sleep_period(10000);
    arm_ptr_->shut_down();
    canfd_close(can_channel_, 0);
    return RetCode::kSuccess;
}

RetCode DualV2_1Arm::setMode(ControlType mode){
    if (mode == ControlType::kVelocity) return RetCode::kInvalidInput;  //Disable Velocity Control
    if (current_mode_ == mode) return RetCode::kSuccess;                //Already in the desired mode
    CONTROL_MODE op_mode = convertOperateMode(mode);
    bool success = (arm_ptr_->set_control_mode(op_mode) == 0);
    if (success) current_mode_ = mode;
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

// Won't use and update current_mode_
RetCode DualV2_1Arm::setMode(int joint_i, ControlType mode){
    if (mode == ControlType::kVelocity) return RetCode::kInvalidInput;  //Disable Velocity Control
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    CONTROL_MODE op_mode = convertOperateMode(mode);
    bool success = (arm_ptr_->set_control_mode(joint_i, op_mode) == 0);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV2_1Arm::setCommand(int joint_i, float val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    if (current_mode_ == ControlType::kPosition){
        arm_ptr_->set_mdh_position(joint_i, val);
    } else if (current_mode_ == ControlType::kVelocity){
        arm_ptr_->set_velocity(joint_i, val);
    }else if (current_mode_ == ControlType::kTorque){
        arm_ptr_->set_current(joint_i, val);
        // arm_ptr_->set_current(joint_i, 0.0);
    }else{
        std::cerr << "DualV2_1Arm::setCommand: Invalid control mode." << std::endl;
        return RetCode::kFailed;
    }
    return RetCode::kSuccess;
}

RetCode DualV2_1Arm::getPosition(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    *val = arm_ptr_->get_mdh_position(joint_i);
    return RetCode::kSuccess;
}

RetCode DualV2_1Arm::getVelocity(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    *val = arm_ptr_->get_velocity(joint_i);
    return RetCode::kSuccess;
}

RetCode DualV2_1Arm::getTorque(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    *val = arm_ptr_->get_current(joint_i);
    return RetCode::kSuccess;
}

RetCode DualV2_1Arm::getState(int joint_i, float* pos, float* vel, float* tor, int read_delay_us){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    // int ret = arm_ptr_->read_status(joint_i);

    if (pos) *pos = (float)arm_ptr_->get_mdh_position(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (vel) *vel = (float)arm_ptr_->get_velocity(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (tor) *tor = (float)arm_ptr_->get_current(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);
    return RetCode::kSuccess;
}

RetCode DualV2_1Arm::getState(std::vector<float>* pos, std::vector<float>* vel, std::vector<float>* tor, int read_delay_us){
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;
        arm_ptr_->read_status(joint_i); //Assume we are not send new cmd, need to update status manually
    }
    return ArmBase::getState(pos, vel, tor, read_delay_us);
}

RetCode DualV2_1Arm::enableMotor(int joint_i, bool enable){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    if (enable_motors_[joint_i] == enable) return RetCode::kSuccess;
    enableMotorInternal(joint_i, enable);
    bool success = isAllBrakeOpened(5000000);
    if (current_mode_ != ControlType::kInvalid) success &= (setMode(joint_i, current_mode_) == RetCode::kSuccess);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV2_1Arm::enableMotors(const std::vector<bool>& enable){
    if (!isValidInput(enable)) return RetCode::kInvalidInput;

    bool has_updated = false;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || enable_motors_[joint_i] == enable[joint_i]) continue;
        enableMotorInternal(joint_i, enable[joint_i]);
        has_updated = true;
    }

    if (has_updated){
        bool success = isAllBrakeOpened(5000000);
        if (current_mode_ != ControlType::kInvalid) success &= (setMode(current_mode_) == RetCode::kSuccess);
        return success ? RetCode::kSuccess : RetCode::kFailed;
    }
    return RetCode::kSuccess;
}

RetCode DualV2_1Arm::enableMotorInternal(int joint_i, bool enable){
    enable_motors_[joint_i] = enable;
    bool success = true;
    if (enable){
        success &= (arm_ptr_->enable_motor(joint_i, true) == 0);
        //Must set to position mode first after disable/shutdown
        success &= (setMode(joint_i, ControlType::kPosition) == RetCode::kSuccess);
    }else{
        success &= (arm_ptr_->enable_motor(joint_i, false) == 0);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

bool DualV2_1Arm::isAllBrakeOpened(long timeout_us){
    bool all_brakes_opened = false;
    long time_left = timeout_us;
    long delay_us = 10000;
    while (!all_brakes_opened && time_left > 0){
        all_brakes_opened = true;
        for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
            if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue; //Continue if this motor is not existing
            arm_ptr_->read_status(joint_i);
            all_brakes_opened &= arm_ptr_->is_brake_opened(joint_i);
        }
        if (all_brakes_opened) break; //Stop immediately without sleep

        sleep_period(delay_us); //10ms
        time_left -= delay_us;
    }
    return all_brakes_opened;
}

CONTROL_MODE DualV2_1Arm::convertOperateMode(ControlType mode){
    if (mode == ControlType::kPosition){
        return CONTROL_MODE::pos_control;
    }else if (mode == ControlType::kVelocity){
        return CONTROL_MODE::vel_control;
    }else if (mode == ControlType::kTorque){ 
        return CONTROL_MODE::cur_control;
    }else{
        throw std::invalid_argument("DualV2_1Arm::convertOperateMode: Invalid control mode.");
    }
}

RetCode DualV2_1Arm::updateEncoderZeroOffset(int joint_i, float offset){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    arm_ptr_->update_encoder_zero_offset(joint_i, offset);
    return RetCode::kSuccess;
}