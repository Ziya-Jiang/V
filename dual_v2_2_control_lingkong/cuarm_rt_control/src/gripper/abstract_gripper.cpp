#include "abstract_gripper.h"

#include <cmath>
#include <iostream>

AbstractGripper::AbstractGripper(const std::string& name, int joint_size, const std::vector<std::array<float, 2>>& joint_position_limits)
    :kName_(name), kJointSize_(joint_size), joint_hardware_limits_(joint_position_limits), last_set_command_(joint_size, -1.0f) {}

RetCode AbstractGripper::setNormalizedPosition(const std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    std::vector<float> denormalized_pos(kJointSize_);
    for (int i = 0; i < kJointSize_; ++i){
        float min_limit = joint_hardware_limits_[i][0];
        float max_limit = joint_hardware_limits_[i][1];
        denormalized_pos[i] = val[i] * (max_limit - min_limit) + min_limit;
    }
    return setPosition(denormalized_pos);
}

RetCode AbstractGripper::getNormalizedPosition(std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    std::vector<float> current_pos(kJointSize_);
    if (getPosition(current_pos) != RetCode::kSuccess) return RetCode::kFailed;
    for (int i = 0; i < kJointSize_; ++i){
        float min_limit = joint_hardware_limits_[i][0];
        float max_limit = joint_hardware_limits_[i][1];
        val[i] = (current_pos[i] - min_limit) / (max_limit - min_limit);
    }
    return RetCode::kSuccess;
}

std::vector<float> AbstractGripper::convertPanelHardware(const std::vector<float>& val){
    return val;
};

RetCode AbstractGripper::setPanelCommand(const std::vector<float>& val){
    std::vector<float> val1 = convertPanelHardware(val);
    return setNormalizedPosition(val1);
}

RetCode AbstractGripper::getPanelPos(std::vector<float>& val){
    if (getNormalizedPosition(val) == RetCode::kSuccess){
        val = convertPanelHardware(val);
        return RetCode::kSuccess;
    }
    return RetCode::kFailed;
}

int AbstractGripper::getJointSize() const{
    return kJointSize_;
}

bool AbstractGripper::isValidInput(const std::vector<float>& val) const{
    return val.size() == kJointSize_;
}

bool AbstractGripper::isDuplicateCommand(const std::vector<float>& val) const{
    if (!avoid_duplicate_command_) return false;
    for (int i = 0; i < kJointSize_; ++i){  
        if (val[i] != last_set_command_[i]){
            return false;
        }
    }
    return true;
}

std::vector<float> AbstractGripper::roundCommand(const std::vector<float>& val) const{
    if (round_command_digits_ < 0) return val;
    float factor = std::pow(10, round_command_digits_);
    std::vector<float> rounded_val(kJointSize_);
    for (int i = 0; i < kJointSize_; ++i){
        rounded_val[i] = int(std::round(val[i] / factor)) * factor;
    }
    return rounded_val;
}

void AbstractGripper::setJointHardwareLimit(const std::vector<float>& min_val, const std::vector<float>& max_val){
    if (min_val.size() != kJointSize_ || max_val.size() != kJointSize_){
        std::cerr << "Error: setJointHardwareLimit input size does not match joint size." << std::endl;
        return;
    }

    for (int i = 0; i < kJointSize_; ++i) {
        joint_hardware_limits_[i][0] = min_val[i];
        joint_hardware_limits_[i][1] = max_val[i];
    }
}

std::vector<float> AbstractGripper::getLastSetCommand() const{
    return last_set_command_;
}