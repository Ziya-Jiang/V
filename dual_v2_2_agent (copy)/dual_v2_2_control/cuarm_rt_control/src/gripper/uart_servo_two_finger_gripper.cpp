#include "uart_servo_two_finger_gripper.h"

UartServoTwoFingerGripper::UartServoTwoFingerGripper(const std::string& name, const std::string& com_id, int id, float time_out):  
    AbstractGripper(name, kDefaultJointSize_, 
    std::vector<std::array<float, 2>>(kDefaultJointPositionLimits_.begin(), kDefaultJointPositionLimits_.end())),
    com_id_(com_id), id_(id), time_out_(time_out){
    servo_shared_ptr_ = std::make_shared<serial_uart_servo>();
    is_open_shared_ptr_ = std::make_shared<bool>(false);
}

UartServoTwoFingerGripper::UartServoTwoFingerGripper(const std::string& name, UartServoTwoFingerGripper& other, int id):  
    AbstractGripper(name, kDefaultJointSize_, 
    std::vector<std::array<float, 2>>(kDefaultJointPositionLimits_.begin(), kDefaultJointPositionLimits_.end())), 
    id_(id){
    com_id_ = other.com_id_;
    time_out_ = other.time_out_;
    servo_shared_ptr_ = other.servo_shared_ptr_;
    is_open_shared_ptr_ = other.is_open_shared_ptr_;
    last_set_command_[0] = other.last_set_command_[0];
}

UartServoTwoFingerGripper::~UartServoTwoFingerGripper(){
    if (*is_open_shared_ptr_)
        this->close();
}

RetCode UartServoTwoFingerGripper::open(){
    if (*is_open_shared_ptr_) return RetCode::kSuccess;
    *is_open_shared_ptr_ = (serial_uart_servo_open(servo_shared_ptr_.get(), com_id_.c_str(), kBaudrate_) == 0);
    return *is_open_shared_ptr_ ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode UartServoTwoFingerGripper::close(){
    *is_open_shared_ptr_ = false; // Mark as closed regardless of the result
    return serial_uart_servo_close(servo_shared_ptr_.get()) == 0 ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode UartServoTwoFingerGripper::setPosition(const std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    std::vector<float> rounded_val = roundCommand(val);
    if (isDuplicateCommand(rounded_val)) return RetCode::kSuccess;

    if (serial_uart_servo_set_position(servo_shared_ptr_.get(), id_, rounded_val[0], time_out_) == 0){
        last_set_command_[0] = rounded_val[0];
        return RetCode::kSuccess;
    }
    return RetCode::kFailed;
}

RetCode UartServoTwoFingerGripper::getPosition(std::vector<float>& val){
    if (!isValidInput(val)) return RetCode::kInvalidInput;
    val[0] = last_set_command_[0];
    return RetCode::kSuccess;
}

std::vector<float> UartServoTwoFingerGripper::convertPanelHardware(const std::vector<float>& val){
    std::vector<float> val1 = val;
    for (int i = 0; i < kJointSize_; ++i){
        val1[i] = 1.0f - val[i];
    }
    return val1;
}
