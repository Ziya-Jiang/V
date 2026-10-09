#ifndef DUAL_V1_H
#define DUAL_V1_H

#include <array>
#include <memory>
#include <string>

#include "hardware_modules/arm_base/arm_base.h"

#ifdef ARCHITECTURE_ARM64
#include "hardware_modules/eyou_dual_arm-multi_can/include/arm64/eu_harmonic.h"
#elif defined(ARCHITECTURE_X86)
#include "hardware_modules/eyou_dual_arm-multi_can/include/x86/eu_harmonic.h"
#else
#error "Unsupported architecture"
#endif
#include "hardware_modules/eyou_dual_arm-multi_can/include/eyou_joints.h"

//Using eyou_dual_arm-multi_can(Support Multi Can)
class DualV1Arm : public ArmBase{
    public:
        DualV1Arm(const std::string& name, int arm_i);
        virtual RetCode open() override;
        virtual RetCode close() override;
        virtual RetCode setMode(ControlType mode) override;
        virtual RetCode setCommand(int joint_i, float val) override;
        virtual RetCode getPosition(int joint_i, float* val) override;
        virtual RetCode getVelocity(int joint_i, float* val) override;
        virtual RetCode getTorque(int joint_i, float* val) override;
        virtual RetCode enableMotor(int joint_i, bool enable) override;
        virtual RetCode setCommandAndGetState(const std::vector<float>& set_val, std::vector<float>* pos=nullptr, 
            std::vector<float>* vel=nullptr, std::vector<float>* tor=nullptr, 
            int set_delay_us=0, int read_delay_us=0) override;
        virtual RetCode updateEncoderZeroOffset(int joint_i, float offset) override;
    private:
        RetCode setMode(int joint_i, ControlType mode);
        harmonic_OperateMode convertOperateMode(ControlType mode);

        const std::string kConfigPath_ = "hardware_modules/eyou_dual_arm-multi_can/config.yaml";
        static constexpr float kPulse2Deg_ = 0.00549316f;
        static constexpr int kDefaultJointSize_ = 7;
        const int kArmIndex_;
        std::array<int, kDefaultJointSize_> sign_;
        std::array<int, kDefaultJointSize_> gear_ratio_;
        std::unique_ptr<EyouJoints> arm_ptr_;
        int can_channel_;
        int pdo_auto_report_ms_;
        bool mode_changed_ = false;
};

#endif