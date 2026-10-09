#include "arm_base.h"

#include <iostream>

#include "robot_platform_utils/cpp/include/time_utils.h"  //for sleep_period

ArmBase::ArmBase(const std::string& name, int joint_size): kName_(name), kJointSize_(joint_size){
    enable_motors_.assign(kJointSize_, false);
    existing_motors_.assign(kJointSize_, false);
}

RetCode ArmBase::setCommand(const std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    bool success = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;
        success &= (setCommand(joint_i, val[joint_i]) == RetCode::kSuccess);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmBase::getState(int joint_i, float* pos, float* vel, float* tor, int read_delay_us){
    if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) return RetCode::kInvalidIndex;
    bool success = true;
    if (pos) success &= (getPosition(joint_i, pos) == RetCode::kSuccess);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (vel) success &= (getVelocity(joint_i, vel) == RetCode::kSuccess);
    if (read_delay_us > 0) sleep_period(read_delay_us);

    if (tor) success &= (getTorque(joint_i, tor) == RetCode::kSuccess);
    if (read_delay_us > 0) sleep_period(read_delay_us);
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmBase::getState(std::vector<float>* pos, std::vector<float>* vel, std::vector<float>* tor, int read_delay_us){
    bool need_read_pos = (pos != nullptr && isValidInput(*pos));
    bool need_read_vel = (vel != nullptr && isValidInput(*vel));
    bool need_read_tor = (tor != nullptr && isValidInput(*tor));
    
    bool success = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;

        //Read motor status
        float* pos_ptr = (need_read_pos)? &((*pos)[joint_i]) : nullptr;
        float* vel_ptr = (need_read_vel)? &((*vel)[joint_i]) : nullptr;
        float* tor_ptr = (need_read_tor)? &((*tor)[joint_i]) : nullptr;
        success &= (getState(joint_i, pos_ptr, vel_ptr, tor_ptr, read_delay_us) == RetCode::kSuccess);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmBase::setCommandAndGetState(const std::vector<float>& set_val, std::vector<float>* pos, 
    std::vector<float>* vel, std::vector<float>* tor, int set_delay_us, int read_delay_us){
    //Possible optimization(verified on DualV1 only): set & read the same joint at once to reduce overhead
    if (!isValidInput(set_val)) return RetCode::kInvalidInput;
    bool need_read_pos = (pos != nullptr && isValidInput(*pos));
    bool need_read_vel = (vel != nullptr && isValidInput(*vel));
    bool need_read_tor = (tor != nullptr && isValidInput(*tor));
    
    bool success = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;

        //Set motor command
        success &= (setCommand(joint_i, set_val[joint_i]) == RetCode::kSuccess);
        if (set_delay_us > 0) sleep_period(set_delay_us);

        //Read motor status
        float* pos_ptr = (need_read_pos)? &((*pos)[joint_i]) : nullptr;
        float* vel_ptr = (need_read_vel)? &((*vel)[joint_i]) : nullptr;
        float* tor_ptr = (need_read_tor)? &((*tor)[joint_i]) : nullptr;
        success &= (getState(joint_i, pos_ptr, vel_ptr, tor_ptr, read_delay_us) == RetCode::kSuccess);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmBase::getPosition(std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    bool success = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;
        success &= (getPosition(joint_i, &val[joint_i]) == RetCode::kSuccess);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmBase::getVelocity(std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    bool success = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;
        success &= (getVelocity(joint_i, &val[joint_i]) == RetCode::kSuccess);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode ArmBase::getTorque(std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    bool success = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!isValidIdx(joint_i) || !enable_motors_[joint_i]) continue;
        success &= (getTorque(joint_i, &val[joint_i]) == RetCode::kSuccess);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

int ArmBase::getJointSize() const{
    return kJointSize_;
}

bool ArmBase::isMotorEnabled(int joint_i) const{
    if (!isValidIdx(joint_i)) return false;
    return enable_motors_[joint_i];
}

RetCode ArmBase::enableMotor(int joint_i, bool enable){
    if (!isValidIdx(joint_i)) return RetCode::kInvalidIndex;
    enable_motors_[joint_i] = enable;
    return RetCode::kSuccess;
}

RetCode ArmBase::enableMotors(const std::vector<bool>& enable){
    if (!isValidInput(enable)) return RetCode::kInvalidInput;
    bool ok = true;
    for (int joint_i=0; joint_i < kJointSize_; joint_i++){
        if (!existing_motors_[joint_i]) continue;
        if (enableMotor(joint_i, enable[joint_i]) != RetCode::kSuccess) ok = false;
    }
    return ok ? RetCode::kSuccess : RetCode::kFailed;
}

bool ArmBase::isValidIdx(int joint_i) const{
    if (joint_i < 0 || joint_i >= kJointSize_ || !existing_motors_[joint_i]){
        return false;
    }
    return true;
}
