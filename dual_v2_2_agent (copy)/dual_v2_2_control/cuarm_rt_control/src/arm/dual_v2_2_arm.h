#ifndef DUAL_V2_2_H
#define DUAL_V2_2_H

#include <array>
#include <memory>
#include <string>

#include "hardware_modules/arm_base/arm_base.h"

#include "hardware_modules/realman_dual_arm/include/robot_arm.h"
#include "hardware_modules/realman_dual_arm/include/motor.h" //For CONTROL_MODE

//Using realman_dual_arm
class DualV2_2Arm : public ArmBase{
    public:
        DualV2_2Arm(const std::string& name, int arm_i,
                    RobotArms::StopRequested stop_requested = {});
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

        const std::string kConfigPath_ = "hardware_modules/realman_dual_arm/config1.yaml";
        static constexpr int kDefaultJointSize_ = 7;
        const int kArmIndex_;
        std::unique_ptr<RobotArms> arm_ptr_;
        int can_channel_;
};

#endif