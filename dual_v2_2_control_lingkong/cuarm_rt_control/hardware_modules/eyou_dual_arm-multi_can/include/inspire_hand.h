#ifndef INSPIRE_HAND_H
#define INSPIRE_HAND_H

#include "eu_canable.h"
#include "eu_harmonic.h"
#include <iostream>
#include <unordered_map>
#include <array>

class InspireHand {
public:
    // 构造函数支持双ID初始化
    InspireHand(int* hand_ids, int *channel); 
    ~InspireHand();

    // 核心功能接口
    void setAngles(int hand_idx, const int16_t angles[6]);
    void setForce(int hand_idx, const int16_t forces[6]);
    void setSpeed(int hand_idx, const int16_t speeds[6]);
    int getAngles(int hand_idx);
    int getForce(int hand_idx);
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
    static void staticReceiveCallback(int dev_index, const harmonic_CanMsg* msg); 
private:
    // 实例成员
    static int hand_ids_[2];  // 存储两个手部ID
    static std::array<harmonic_CanMsg, 2> rx_msgs_; // 每个手的响应缓存
    static std::array<bool, 2> response_flags_;     // 响应到达标志
    int can_channel_[2]; // CAN通道

    // 寄存器地址（保持原框架）
    enum Registers {
        SPEED_SET_ADDR   = 0x05F2,
        FORCE_SET_ADDR   = 0x05DA,
        ANGLE_SET_ADDR   = 0x05CE,
        ANGLE_ACT_ADDR   = 0x060A,
        FORCE_ACT_ADDR   = 0x062E,
        CURRENT_ACT_ADDR = 0x063A,

        CURRENT_LIMIT_ADDR = 0x03FC,
        DEFAULT_FORCE_SET = 0x0414,

        CLEAR_ERROR_ADDR = 0x03EC,
        ERROR_ADDR       = 0x0646,
        STATUS_ADDR      = 0x064C,

        GESTURE_FORCE_CLB_ADDR = 0x03F1
    };

    // CAN协议工具函数（保持原框架）
    uint32_t build_ext_id(unsigned hand_id, unsigned reg_addr, bool is_write, bool is_wrist);
    int send_can_frame(int hand_idx, const canable_CanMsg &msg, harmonic_CanMsg &response, int timeout = 10);
    int write_register(int hand_idx, unsigned reg_addr, bool is_wrist, int16_t value);
    int16_t read_register(int hand_idx, unsigned reg_addr, bool is_wrist);
    int write_multi_param(int hand_idx, unsigned reg_addr, bool is_wrist, const int16_t *values, int length, u_char size);
    int read_multi_param(int hand_idx, unsigned reg_addr, bool is_wrist, short* result, int length, u_char size);
};

#endif
