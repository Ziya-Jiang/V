#include "inspire_hand_wrapper.h"

#include "src/file_utils.h"
#include "hardware_modules/eyou_dual_arm-multi_can/include/callback_dispatcher.h"
#include "hardware_modules/eyou_dual_arm-multi_can/include/config_loader.h"

std::unique_ptr<InspireHand> InspireHandWrapper::hand_unique_ptr_;
int InspireHandWrapper::can_channel_[2];
bool InspireHandWrapper::opened_ = false;

InspireHandWrapper::InspireHandWrapper(const std::string& name, int hand_id): AbstractGripper(name, kDefaultJointSize_, 
    std::vector<std::array<float, 2>>(kDefaultJointPositionLimits_.begin(), kDefaultJointPositionLimits_.end())),
    id_(hand_id) {
    if (!hand_unique_ptr_){
        ConfigLoader config_loader(get_absolute_path(kConfigPath_));
        RobotConfig& robot_config = config_loader.getRobotConfig();
        int hand_ids[2] = {robot_config.hands[0].id, robot_config.hands[1].id};
        can_channel_[0] = robot_config.motors[0].can_channel;
        can_channel_[1] = robot_config.motors[1].can_channel;
        hand_unique_ptr_ = std::make_unique<InspireHand>(hand_ids, can_channel_);
    }
    round_command_digits_ = 1; //Round to tens place
}

RetCode InspireHandWrapper::open() {
    if (opened_) return RetCode::kSuccess;
    canbox_init({can_channel_[0], can_channel_[1]});
    CallbackDispatcher::init();
    opened_ = true;
    return RetCode::kSuccess;
}

RetCode InspireHandWrapper::close() {
    if (!opened_) return RetCode::kSuccess;
    canbox_close({can_channel_[0], can_channel_[1]});
    opened_ = false;
    return RetCode::kSuccess;
}

RetCode InspireHandWrapper::setPosition(const std::vector<float>& val) {
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    std::vector<float> rounded_val = roundCommand(val);
    if (isDuplicateCommand(rounded_val)) return RetCode::kSuccess; //Consider duplicate command as success without sending to hardware

    int16_t angles[6];
    for (int j = 0; j < kJointSize_; ++j) {
        angles[j] = static_cast<int16_t>(rounded_val[j]);
        last_set_command_[j] = rounded_val[j];
    }
    hand_unique_ptr_->setAngles(id_, angles);
    return RetCode::kSuccess;
}

RetCode InspireHandWrapper::getPosition(std::vector<float>& val) {
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    bool success = (hand_unique_ptr_->getAngles(id_) == CANABLE_SUCCESS);
    for (int j = 0; j < kJointSize_; ++j) {
        val[j] = static_cast<float>(hand_unique_ptr_->joint_angle_[id_][j]);
    }
    return success ? RetCode::kSuccess : RetCode::kFailed;
}

std::vector<float> InspireHandWrapper::convertPanelHardware(const std::vector<float>& val){
    std::vector<float> val1 = val;
    for (int i = 0; i < kJointSize_; ++i){
        val1[i] = 1.0f - val[i];
    }
    
    //Panel use urdf order: "thumbRotate", "thumbFlex", "indexFlex", "middleFlex", "ringFlex", "littleFlex"
    //Hardware api order: "littleFlex", "ringFlex", "middleFlex", "indexFlex", "thumbFlex", "thumbRotate" 
    std::swap(val1[0], val1[5]);
    std::swap(val1[1], val1[4]);
    std::swap(val1[2], val1[3]);
    return val1;
}