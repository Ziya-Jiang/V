#ifndef UART_SERVO_TWO_FINGER_GRIPPER_H
#define UART_SERVO_TWO_FINGER_GRIPPER_H

#include <string>
#include <memory>
#include <string>

#include "src/gripper/abstract_gripper.h"
extern "C" 
{
    #include "hardware_modules/serial_uart_servo/serial_uart_servo.h" 
}

class UartServoTwoFingerGripper: public AbstractGripper {
    public:
        UartServoTwoFingerGripper(const std::string& name, const std::string& com_id, int id, float time_out = 1000.0f);
        UartServoTwoFingerGripper(const std::string& name, UartServoTwoFingerGripper& other, int id);
        ~UartServoTwoFingerGripper();
        RetCode open() override;
        RetCode close() override;
        RetCode setPosition(const std::vector<float>& val) override;
        RetCode getPosition(std::vector<float>& val) override;
        std::vector<float> convertPanelHardware(const std::vector<float>& val) override;
    private:
        static constexpr int kDefaultJointSize_ = 1;
        static constexpr std::array<std::array<float, 2>, kDefaultJointSize_> kDefaultJointPositionLimits_ = {{500.0f, 3000.0f}};
        const int kBaudrate_ = 115200;
        std::shared_ptr<serial_uart_servo> servo_shared_ptr_;
        std::shared_ptr<bool> is_open_shared_ptr_;
        std::string com_id_;
        int id_;
        float time_out_;
};

#endif