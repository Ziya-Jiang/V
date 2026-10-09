#ifndef SPARK2_V1_H
#define SPARK2_V1_H

#include <array>
#include <memory>
#include <string>

#include "hardware_modules/arm_base/arm_base.h"
#include "hardware_modules/single_arm_v1/include/robot_arm.h"
#include "hardware_modules/single_arm_v1/include/motor.h" //For CONTROL_MODE

class ArmV1 : public ArmBase{
    public:
        ArmV1(const std::string& name);
        virtual RetCode open() override;
        virtual RetCode close() override;
        virtual RetCode setMode(ControlType mode) override;
        virtual RetCode setCommand(int joint_i, float val) override;
        virtual RetCode getPosition(int joint_i, float* val) override;
        virtual RetCode getVelocity(int joint_i, float* val) override;
        virtual RetCode getTorque(int joint_i, float* val) override;
        virtual RetCode enableMotor(int joint_i, bool enable) override;
        virtual RetCode enableMotors(const std::vector<bool>& enable) override;
        virtual RetCode getState(int joint_i, float* pos=nullptr, float* vel=nullptr, float* tor=nullptr, int read_delay_us=0) override;
        virtual RetCode getState(std::vector<float>* pos=nullptr, std::vector<float>* vel=nullptr, std::vector<float>* tor=nullptr, 
            int read_delay_us=0) override;
        virtual RetCode updateEncoderZeroOffset(int joint_i, float offset) override;
    private:
        RetCode setMode(int joint_i, ControlType mode);
        CONTROL_MODE convertOperateMode(ControlType mode);
        RetCode enableMotorInternal(int joint_i, bool enable);
        bool isAllBrakeOpened(long timeout_us);

        const std::string kConfigPath_ = "hardware_modules/single_arm_v1/config.yaml";
        static constexpr int kDefaultJointSize_ = 6;
        std::vector<double> last_read_pos_{std::vector<double>(kDefaultJointSize_)};
        std::vector<double> last_read_vel_{std::vector<double>(kDefaultJointSize_)};
        std::vector<double> tau_comp_{std::vector<double>(kDefaultJointSize_)};
        std::unique_ptr<RobotArm> arm_ptr_;
        int can_channel_;
};

#endif