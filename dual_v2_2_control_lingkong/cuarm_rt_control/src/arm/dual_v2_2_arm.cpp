#include "dual_v2_2_arm.h"

#include "src/file_utils.h"
#include "hardware_modules/realman_dual_arm/include/config_loader.h"
#include "hardware_modules/realman_dual_arm/include/can_fd.h"
#include <algorithm>
#include <utility>

DualV2_2Arm::DualV2_2Arm(const std::string& name, int arm_i,
                         RobotArms::StopRequested stop_requested)
    : ArmBase(name, kDefaultJointSize_), kArmIndex_(arm_i) {
    ConfigLoader config_loader(get_absolute_path(kConfigPath_));
    const auto& robot_config = config_loader.getRobots()[arm_i];
    arm_ptr_ = std::make_unique<RobotArms>(
        arm_i, robot_config, std::move(stop_requested));
    can_channel_ = robot_config.motors.can_channel;
    //Load parameters
    for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
        enable_motors_[joint_i] = (bool)robot_config.motors.enabled[joint_i];
        existing_motors_[joint_i] = (bool)robot_config.motors.enabled[joint_i];
    }
}

RetCode DualV2_2Arm::open(){
    if (canfd_init(can_channel_, 0) != 0) {
        std::cerr << "[CANMAP] Refusing to initialize arm " << kArmIndex_
                  << ": fixed USBCANFD serial binding failed." << std::endl;
        return RetCode::kFailed;
    }
    if (arm_ptr_->initialize() != 0) {
        canfd_close(can_channel_, 0);
        return RetCode::kFailed;
    }
    //Must set to position mode first after disable/shutdown
    if (setMode(ControlType::kPosition) != RetCode::kSuccess) {
        return RetCode::kFailed;
    }
    return isAllBrakeOpened(5000000)
        ? RetCode::kSuccess
        : RetCode::kFailed;
}

RetCode DualV2_2Arm::close(){
    bool success = (setMode(ControlType::kPosition) == RetCode::kSuccess);
    sleep_period(10000);
    success &= (arm_ptr_->shut_down() == 0);
    canfd_close(can_channel_, 0);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV2_2Arm::setMode(ControlType mode){
    if (mode == ControlType::kVelocity) return RetCode::kInvalidInput;  //Disable Velocity Control
    if (current_mode_ == mode) return RetCode::kSuccess;                //Already in the desired mode
    CONTROL_MODE op_mode = convertOperateMode(mode);
    bool success = (arm_ptr_->set_control_mode(op_mode) == 0);
    if (success) current_mode_ = mode;
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

// Won't use and update current_mode_
RetCode DualV2_2Arm::setMode(int joint_i, ControlType mode){
    if (mode == ControlType::kVelocity) return RetCode::kInvalidInput;  //Disable Velocity Control
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    CONTROL_MODE op_mode = convertOperateMode(mode);
    bool success = (arm_ptr_->set_control_mode(joint_i, op_mode) == 0);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV2_2Arm::setCommand(int joint_i, float val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    int ret = -1;
    if (current_mode_ == ControlType::kPosition){
        ret = arm_ptr_->set_mdh_position(joint_i, val);
    } else if (current_mode_ == ControlType::kVelocity){
        ret = arm_ptr_->set_velocity(joint_i, val);
    }else if (current_mode_ == ControlType::kTorque){
        ret = arm_ptr_->set_current(joint_i, val);
    }else{
        std::cerr << "DualV2_2Arm::setCommand: Invalid control mode." << std::endl;
        return RetCode::kFailed;
    }
    return ret == 0 ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV2_2Arm::getPosition(int joint_i, float* val){
    if (!val) return RetCode::kInvalidInput;
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    if (arm_ptr_->read_status(joint_i) != 0) return RetCode::kFailed;
    *val = arm_ptr_->get_mdh_position(joint_i);
    return RetCode::kSuccess;
}

RetCode DualV2_2Arm::getVelocity(int joint_i, float* val){
    if (!val) return RetCode::kInvalidInput;
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    if (arm_ptr_->read_status(joint_i) != 0) return RetCode::kFailed;
    *val = arm_ptr_->get_velocity(joint_i);
    return RetCode::kSuccess;
}

RetCode DualV2_2Arm::getTorque(int joint_i, float* val){
    if (!val) return RetCode::kInvalidInput;
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    if (arm_ptr_->read_status(joint_i) != 0) return RetCode::kFailed;
    *val = arm_ptr_->get_current(joint_i);
    return RetCode::kSuccess;
}

RetCode DualV2_2Arm::getState(int joint_i, float* pos, float* vel, float* tor, int read_delay_us){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    if (arm_ptr_->read_status(joint_i) != 0) return RetCode::kFailed;

    if (pos) *pos = (float)arm_ptr_->get_mdh_position(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (vel) *vel = (float)arm_ptr_->get_velocity(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (tor) *tor = (float)arm_ptr_->get_current(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);
    return RetCode::kSuccess;
}

RetCode DualV2_2Arm::getState(std::vector<float>* pos, std::vector<float>* vel, std::vector<float>* tor, int read_delay_us){
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;
        if (arm_ptr_->read_status(joint_i) != 0) return RetCode::kFailed;
    }
    return ArmBase::getState(pos, vel, tor, read_delay_us);
}

RetCode DualV2_2Arm::enableMotor(int joint_i, bool enable){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    if (enable_motors_[joint_i] == enable) return RetCode::kSuccess;
    if (enableMotorInternal(joint_i, enable) != RetCode::kSuccess) {
        return RetCode::kFailed;
    }
    if (!enable) {
        return RetCode::kSuccess;
    }
    bool success = isAllBrakeOpened(5000000);
    if (current_mode_ != ControlType::kInvalid) success &= (setMode(joint_i, current_mode_) == RetCode::kSuccess);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode DualV2_2Arm::enableMotors(const std::vector<bool>& enable){
    if (!isValidInput(enable)) return RetCode::kInvalidInput;

    bool has_updated = false;
    bool update_success = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || enable_motors_[joint_i] == enable[joint_i]) continue;
        update_success &=
            enableMotorInternal(joint_i, enable[joint_i]) == RetCode::kSuccess;
        has_updated = true;
    }

    if (has_updated){
        if (!update_success) {
            return RetCode::kFailed;
        }
        const bool any_enabled =
            std::any_of(enable.begin(), enable.end(), [](bool value) {
                return value;
            });
        if (!any_enabled) {
            return RetCode::kSuccess;
        }
        bool success = isAllBrakeOpened(5000000);
        if (current_mode_ != ControlType::kInvalid) success &= (setMode(current_mode_) == RetCode::kSuccess);
        return success ? RetCode::kSuccess : RetCode::kFailed;
    }
    return RetCode::kSuccess;
}

RetCode DualV2_2Arm::enableMotorInternal(int joint_i, bool enable){
    bool success = true;
    if (enable){
        success &= (arm_ptr_->enable_motor(joint_i, true) == 0);
        //Must set to position mode first after disable/shutdown
        success &= (setMode(joint_i, ControlType::kPosition) == RetCode::kSuccess);
    }else{
        success &= (arm_ptr_->enable_motor(joint_i, false) == 0);
    }
    if (success) enable_motors_[joint_i] = enable;
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

bool DualV2_2Arm::isAllBrakeOpened(long timeout_us){
    bool all_brakes_opened = false;
    long time_left = timeout_us;
    long delay_us = 10000;
    while (!all_brakes_opened && time_left > 0){
        if (arm_ptr_->stopRequested()) {
            return false;
        }
        all_brakes_opened = true;
        for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
            if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue; //Continue if this motor is not existing
            if (arm_ptr_->read_status(joint_i) != 0) return false;
            all_brakes_opened &= arm_ptr_->is_brake_opened(joint_i);
        }
        if (all_brakes_opened) break; //Stop immediately without sleep

        sleep_period(delay_us); //10ms
        time_left -= delay_us;
    }
    return all_brakes_opened;
}

CONTROL_MODE DualV2_2Arm::convertOperateMode(ControlType mode){
    if (mode == ControlType::kPosition){
        return CONTROL_MODE::pos_control;
    }else if (mode == ControlType::kVelocity){
        return CONTROL_MODE::vel_control;
    }else if (mode == ControlType::kTorque){ 
        return CONTROL_MODE::cur_control;
    }else{
        throw std::invalid_argument("DualV2_2Arm::convertOperateMode: Invalid control mode.");
    }
}

RetCode DualV2_2Arm::updateEncoderZeroOffset(int joint_i, float offset){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    arm_ptr_->update_encoder_zero_offset(joint_i, offset);
    return RetCode::kSuccess;
}