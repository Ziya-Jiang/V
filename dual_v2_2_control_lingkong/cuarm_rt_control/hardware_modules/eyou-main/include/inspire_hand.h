#ifndef INSPIRE_HAND_H
#define INSPIRE_HAND_H

#include "hand_can_transport.h"
#include <iostream>
#include <array>
#include <memory>

class InspireHand {
public:
    // 构造函数支持双ID初始化
    InspireHand(int* hand_ids); 
    InspireHand(int* hand_ids, std::shared_ptr<IHandCanTransport> transport);
    ~InspireHand();

    // 核心功能接口
    void setAngles(int hand_idx, const int16_t angles[6]);
    void setForce(int hand_idx, const int16_t forces[6]);
    void setSpeed(int hand_idx, const int16_t speeds[6]);
    int getAngles(int hand_idx);
    int getForces(int hand_idx);
    int getCurrent(int hand_idx);

    int getCurrentLimit(int hand_idx);
    int getForceLimit(int hand_idx);

    int getError(int hand_idx);
    int getStatus(int hand_idx);
    void decodeError();
    void decodeStatus();
    int clearError(int hand_idx);

    int calibrateForce(int hand_idx);
    
    // 数据存储（两个手各6个关节）
    std::array<std::array<short, 6>, 2> joint_angle_; 
    std::array<std::array<short, 6>, 2> joint_force_; 
    std::array<std::array<short, 6>, 2> joint_current_; 

    std::array<std::array<short, 6>, 2> joint_error_; 
    std::array<std::array<short, 6>, 2> joint_status_; 

    std::array<std::array<short, 6>, 2> joint_force_limit_; 
    std::array<std::array<short, 6>, 2> joint_current_limit_; 
private:
    // 实例成员
    int hand_ids_[2];  // 存储两个手部ID
    std::shared_ptr<IHandCanTransport> transport_;

    // 寄存器地址（保持原框架）
    enum Registers {
        SPEED_SET_ADDR   = 0x05F2,
        FORCE_SET_ADDR   = 0x05DA,
        ANGLE_SET_ADDR   = 0x05CE,
        ANGLE_ACT_ADDR   = 0x060A,
        FORCE_ACT_ADDR   = 0x062E,
        CURRENT_ADDR     = 0x063A,

        CURRENT_LIMIT_ADDR = 0x03FC,
        DEFAULT_FORCE_SET = 0x0414,

        CLEAR_ERROR_ADDR = 0x03EC,
        ERROR_ADDR       = 0x0646,
        STATUS_ADDR      = 0x064C,

        GESTURE_FORCE_CLB_ADDR = 0x03F1
    };

    // InspireHand寄存器协议工具函数
    uint32_t build_ext_id(unsigned hand_id, unsigned reg_addr, bool is_write, bool is_wrist);
    int write_register(int hand_idx, unsigned reg_addr, bool is_wrist, int16_t value);
    int16_t read_register(int hand_idx, unsigned reg_addr, bool is_wrist);
    int write_multi_param(int hand_idx, unsigned reg_addr, bool is_wrist, const int16_t *values, int length, uint8_t size);
    int read_multi_param(int hand_idx, unsigned reg_addr, bool is_wrist, short* result, int length, uint8_t size);
};

#endif
