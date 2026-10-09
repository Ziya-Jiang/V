#include "spark2_v1.h"

#include "src/file_utils.h"
#include "hardware_modules/single_arm_v1/include/config_loader.h"
#include "hardware_modules/single_arm_v1/include/can_fd.h"
#include "hardware_modules/single_arm_v1/include/motor.h"

ArmV1::ArmV1(const std::string& name): ArmBase(name, kDefaultJointSize_){
    ConfigLoader config_loader(get_absolute_path(kConfigPath_));
    const auto& robot_config = config_loader.getRobot();
    arm_ptr_ = std::make_unique<RobotArm>(robot_config);
    can_channel_ = robot_config.motors.can_channel;
    //Load parameters
    for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
        enable_motors_[joint_i] = (bool)robot_config.motors.enabled[joint_i];
        existing_motors_[joint_i] = (bool)robot_config.motors.enabled[joint_i];
    }
}

RetCode ArmV1::open(){
    can_normal_init(0);
    canfd_init(can_channel_, 0);
    bool success = (arm_ptr_->initialize(2000) == 0);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmV1::close(){
    setMode(ControlType::kPosition); //To prevent stop at Torque motor for Realman motor since Vel mode is disabled
    sleep_period(10000);
    arm_ptr_->shut_down();
    canfd_close(can_channel_, 0);
    can_normal_close(0);
    return RetCode::kSuccess;
}

RetCode ArmV1::setMode(ControlType mode){
    if (current_mode_ == mode) return RetCode::kSuccess;                //Already in the desired mode
    bool success=true;
    for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
        if (mode == ControlType::kVelocity && arm_ptr_->get_motor_type(joint_i) == MotorType::WHJ)
            continue; //Disable Velocity Control for Realman motor due to its poor performance
        success &= (setMode(joint_i, mode) == RetCode::kSuccess)? true : false;
    }
    if (success) current_mode_ = mode;
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

// Won't use and update current_mode_
RetCode ArmV1::setMode(int joint_i, ControlType mode){
    if (mode == ControlType::kVelocity && arm_ptr_->get_motor_type(joint_i) == MotorType::WHJ) 
        return RetCode::kInvalidInput; //Disable Velocity Control for Reamlman motor due to its poor performance
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    CONTROL_MODE op_mode = convertOperateMode(mode);
    bool success = (arm_ptr_->set_control_mode(joint_i, op_mode) == 0);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmV1::setCommand(int joint_i, float val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    if (current_mode_ == ControlType::kPosition){
        arm_ptr_->set_mdh_position(joint_i, val);
    } else if (current_mode_ == ControlType::kVelocity){
        arm_ptr_->set_velocity(joint_i, val);
    }else if (current_mode_ == ControlType::kTorque){
        arm_ptr_->set_current(joint_i, val);
    }else{
        std::cerr << "ArmV1::setCommand: Invalid control mode." << std::endl;
        return RetCode::kFailed;
    }
    return RetCode::kSuccess;
}

RetCode ArmV1::getPosition(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    *val = arm_ptr_->get_mdh_position(joint_i);
    return RetCode::kSuccess;
}

RetCode ArmV1::getVelocity(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    *val = arm_ptr_->get_velocity(joint_i);
    return RetCode::kSuccess;
}

RetCode ArmV1::getTorque(int joint_i, float* val){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    *val = arm_ptr_->get_current(joint_i);
    return RetCode::kSuccess;
}

RetCode ArmV1::getState(int joint_i, float* pos, float* vel, float* tor, int read_delay_us){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    // int ret = arm_ptr_->read_status(joint_i);
    if (arm_ptr_->get_motor_type(joint_i) == MotorType::RMD) 
        arm_ptr_->read_status(joint_i);

    if (pos) *pos = (float)arm_ptr_->get_mdh_position(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (vel) *vel = (float)arm_ptr_->get_velocity(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (tor) *tor = (float)arm_ptr_->get_current(joint_i);
    if (read_delay_us > 0) sleep_period(read_delay_us);
    return RetCode::kSuccess;
}

RetCode ArmV1::getState(std::vector<float>* pos, std::vector<float>* vel, std::vector<float>* tor, int read_delay_us){
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;
        arm_ptr_->read_status(joint_i); //Assume we are not send new cmd, need to update status manually
    }
    return ArmBase::getState(pos, vel, tor, read_delay_us);
}

RetCode ArmV1::enableMotor(int joint_i, bool enable){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    if (enable_motors_[joint_i] == enable) return RetCode::kSuccess;
    enableMotorInternal(joint_i, enable);
    if (current_mode_ != ControlType::kInvalid) 
        return setMode(joint_i, current_mode_);
    return RetCode::kSuccess;
}

RetCode ArmV1::enableMotors(const std::vector<bool>& enable){
    if (!isValidInput(enable)) return RetCode::kInvalidInput;

    bool has_updated = false;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || enable_motors_[joint_i] == enable[joint_i]) continue;
        enableMotorInternal(joint_i, enable[joint_i]);
        has_updated = true;
    }

    if (has_updated && current_mode_ != ControlType::kInvalid) 
        return setMode(current_mode_);
    return RetCode::kSuccess;
}

RetCode ArmV1::enableMotorInternal(int joint_i, bool enable){
    enable_motors_[joint_i] = enable;
    bool success = true;
    if (enable){
        success &= (arm_ptr_->enable_motor(joint_i, true) == 0);
    }else{
        success &= (arm_ptr_->enable_motor(joint_i, false) == 0);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

bool ArmV1::isAllBrakeOpened(long timeout_us){
    bool all_brakes_opened = false;
    bool brakes_opened[kDefaultJointSize_] = {true};
    long time_left = timeout_us;
    long delay_us = 10000;
    while (!all_brakes_opened && time_left > 0){
        all_brakes_opened = true;
        for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
            if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue; //Continue if this motor is not existing
            arm_ptr_->read_status(joint_i);
            brakes_opened[joint_i] = arm_ptr_->is_brake_opened(joint_i);
            all_brakes_opened &= brakes_opened[joint_i];
        }
        if (all_brakes_opened) break; //Stop immediately without sleep

        sleep_period(delay_us); //10ms
        time_left -= delay_us;
    }

    //Print info
    if (!all_brakes_opened){
        std::cout <<"Arm " << kName_ <<":\n";
        for (int joint_i=0; joint_i < kDefaultJointSize_; joint_i++){
            if (brakes_opened[joint_i]) continue;
            std::cout << "joint " << joint_i << " brake not open\n";
        }
    }
    std::cout <<"all_brakes_openedall_brakes_openedall_brakes_openedall_brakes_opened\n";
    return all_brakes_opened;
}

CONTROL_MODE ArmV1::convertOperateMode(ControlType mode){
    if (mode == ControlType::kPosition){
        return CONTROL_MODE::pos_control;
    }else if (mode == ControlType::kVelocity){
        return CONTROL_MODE::vel_control;
    }else if (mode == ControlType::kTorque){ 
        return CONTROL_MODE::cur_control;
    }else{
        throw std::invalid_argument("ArmV1::convertOperateMode: Invalid control mode.");
    }
}

RetCode ArmV1::updateEncoderZeroOffset(int joint_i, float offset){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    arm_ptr_->update_encoder_zero_offset(joint_i, offset);
    return RetCode::kSuccess;
}