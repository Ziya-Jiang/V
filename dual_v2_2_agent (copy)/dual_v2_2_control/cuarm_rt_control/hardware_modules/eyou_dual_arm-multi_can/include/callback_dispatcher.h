// callback_dispatcher.h
#pragma once
#include "inspire_hand.h"
#include "eyou_joints.h"
#include "torque_sensor.h"
class CallbackDispatcher {
public:
    static void init() {
        harmonic_setReceiveDataCallBack(&globalReceiveCallback);
    }
    
    static void globalReceiveCallback(int dev_index, const harmonic_CanMsg* msg) {
        uint32_t cob_id = msg->cob_id;
        if (cob_id >= 0x15 && cob_id <= 0x28)
        {
            TorSensor::staticReceiveCallback(dev_index, msg);
        }
        // 分发灵巧手消息
        if (cob_id >= 0x8000) {
            InspireHand::staticReceiveCallback(dev_index, msg);
        }
        // 分发关节臂消息
        else if ((cob_id >= 0x180 && cob_id <= 0x180+14) || 
                (cob_id >= 0x280 && cob_id <= 0x280+14)) {
            EyouJoints::staticReceiveCallback(dev_index, msg);
        }
    }
};