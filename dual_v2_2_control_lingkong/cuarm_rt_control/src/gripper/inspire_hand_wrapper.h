#ifndef INSPIRE_HAND_WRAPPER_H
#define INSPIRE_HAND_WRAPPER_H

#include <iostream>
#include <memory>
#include <vector>
#include <string>

#include "src/gripper/abstract_gripper.h"
#include "hardware_modules/eyou_dual_arm-multi_can/include/inspire_hand.h"

//Wrapper of eyou_dual_arm-multi_can's InspireHand class
class InspireHandWrapper: public AbstractGripper {
    public:
        InspireHandWrapper(const std::string& name, int hand_id);
        RetCode open() override;
        RetCode close() override;
        RetCode setPosition(const std::vector<float>& val) override;
        RetCode getPosition(std::vector<float>& val) override;
        std::vector<float> convertPanelHardware(const std::vector<float>& val) override;
    private:
        const std::string kConfigPath_ = "hardware_modules/eyou_dual_arm-multi_can/config.yaml";
        static constexpr int kDefaultJointSize_ = 6;
        static constexpr std::array<std::array<float, 2>, kDefaultJointSize_> kDefaultJointPositionLimits_ = {{
            {{0.0f, 1000.0f}},
            {{0.0f, 1000.0f}},
            {{0.0f, 1000.0f}},
            {{0.0f, 1000.0f}},
            {{0.0f, 1000.0f}},
            {{0.0f, 1000.0f}},
        }};
        static std::unique_ptr<InspireHand> hand_unique_ptr_; //Only one pair of Inspire Hand is allowed currently
        static int can_channel_[2];
        static bool opened_;
        int id_;
};
#endif