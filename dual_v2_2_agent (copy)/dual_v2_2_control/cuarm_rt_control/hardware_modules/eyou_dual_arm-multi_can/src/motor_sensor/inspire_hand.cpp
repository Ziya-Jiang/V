#include "inspire_hand.h"
#include <chrono>
#include <thread>
#include <algorithm>
// 静态成员初始化
int InspireHand::hand_ids_[2] = {};
std::array<harmonic_CanMsg, 2> InspireHand::rx_msgs_ = {}; // 每个手的响应缓存
std::array<bool, 2> InspireHand::response_flags_ = {};     // 响应到达标志

// 构造函数：注册ID到实例映射
InspireHand::InspireHand(int* hand_ids, int *channel) {
    std::cout << "Constructor called with right_id=" << hand_ids[0] << ", left_id=" << hand_ids[1] << std::endl;
    for (int i = 0;i<2;++i)
    {
        hand_ids_[i] = hand_ids[i];
        can_channel_[i] = channel[i];
        response_flags_[i] = false;
    }
    
    // 初始化关节角度
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 6; j++) {
            joint_angle_[i][j] = 0;
        }
    }
    //harmonic_setReceiveDataCallBack(&InspireHand::staticReceiveCallback);
    // 初始化 CAN 通信
}

// 析构函数：清理注册
InspireHand::~InspireHand() {
}

void InspireHand::staticReceiveCallback(int dev_index, const harmonic_CanMsg* msg){
    const harmonic_CanMsg& received_msg = *msg;
    int received_hand_id = received_msg.cob_id & 0x3FFF;
    // std::cout << "[Callback] Calculated hand_id: " << hand_id << std::endl;

    // 查找是否有对应实例
    if (std::find(InspireHand::hand_ids_, InspireHand::hand_ids_ + 2, received_hand_id) != InspireHand::hand_ids_ + 2) {
        // std::cout << "[Callback] Found matching InspireHand instance." << std::endl;

        // InspireHand* instance = it->second;

        int hand_idx = -1;
        if (InspireHand::hand_ids_[0] == received_hand_id) {
            hand_idx = 0;
        } else if (InspireHand::hand_ids_[1] == received_hand_id) {
            hand_idx = 1;
        }

        if (hand_idx >= 0) {
            InspireHand::rx_msgs_[hand_idx] = received_msg;
            InspireHand::response_flags_[hand_idx] = true;

        } else {
            // std::cout << "[Callback] Warning: hand_id matched instance but not found in hand_ids_[]" << std::endl;
        }

    } else {
        // std::cout << "[Callback] Warning: No InspireHand instance found for hand_id: " << hand_id << std::endl;
    }
}
//------------------ 核心功能实现 ------------------//
void InspireHand::setSpeed(int hand_idx, const int16_t speeds[6]) {
    if (hand_idx < 0 || hand_idx > 1) return;
    write_multi_param(hand_idx, SPEED_SET_ADDR, false, speeds, 4, 2);
    write_multi_param(hand_idx, SPEED_SET_ADDR+8, false, speeds+4, 2, 2);
}

void InspireHand::setForce(int hand_idx, const int16_t forces[6]) {
    if (hand_idx < 0 || hand_idx > 1) return;
    write_multi_param(hand_idx, FORCE_SET_ADDR, false, forces, 4, 2);
    write_multi_param(hand_idx, FORCE_SET_ADDR+8, false, forces+4, 2, 2);
}

void InspireHand::setAngles(int hand_idx, const int16_t angles[6]) {
    if (hand_idx < 0 || hand_idx > 1) return;
    write_multi_param(hand_idx, ANGLE_SET_ADDR, false, angles, 4, 2);
    write_multi_param(hand_idx, ANGLE_SET_ADDR+8, false, angles+4, 2, 2);
}

int InspireHand::clearError(int hand_idx) {
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int16_t value[] = {1};
    if (write_multi_param(hand_idx, CLEAR_ERROR_ADDR, false, value, 1, 1) == CANABLE_SUCCESS){
        std::cout << "Successfully Clear Error.\n";
        return 0;
    }
    return -1;
}

int InspireHand::calibrateForce(int hand_idx){
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int16_t value[] = {1};
    if (write_multi_param(hand_idx, GESTURE_FORCE_CLB_ADDR, false, value, 1, 1) != CANABLE_SUCCESS) 
        return -1;
    
    // Wait for the second response confirming calibration completion
    printf("Calibrating...\n");
    int timeout_ms = 25 * 1e3;
    response_flags_[hand_idx] = false;
    auto start = std::chrono::steady_clock::now();
    auto end = start;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() < timeout_ms){
        if (response_flags_[hand_idx]) {
            response_flags_[hand_idx] = false;
            std::cout <<"Calibration completed successfully.\n";
            return 0;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        end = std::chrono::steady_clock::now();
    }
    std::cout <<"Timeout\n";
    return -3;
}

int InspireHand::getAngles(int hand_idx) {
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int ret = 0;
    ret |= read_multi_param(hand_idx, ANGLE_ACT_ADDR, false, joint_angle_[hand_idx].data(), 4, 2);
    ret |= read_multi_param(hand_idx, ANGLE_ACT_ADDR+8, false, joint_angle_[hand_idx].data()+4, 2, 2);
    return ret == 0? 0 : -1;
}

int InspireHand::getForce(int hand_idx) {
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int ret = 0;
    ret |= read_multi_param(hand_idx, FORCE_ACT_ADDR, false, joint_force_[hand_idx].data(), 4, 2);
    ret |= read_multi_param(hand_idx, FORCE_ACT_ADDR+8, false, joint_force_[hand_idx].data()+4, 2, 2);
    return ret == 0? 0 : -1;
}

int InspireHand::getCurrent(int hand_idx){
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int ret = 0;
    ret |= read_multi_param(hand_idx, CURRENT_ACT_ADDR, false, joint_current_[hand_idx].data(), 4, 2);
    ret |= read_multi_param(hand_idx, CURRENT_ACT_ADDR+8, false, joint_current_[hand_idx].data()+4, 2, 2);
    return ret == 0? 0 : -1;
}

int InspireHand::getError(int hand_idx) {
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int ret = 0;
    ret |= read_multi_param(hand_idx, ERROR_ADDR, false, joint_error_[hand_idx].data(), 6, 1);
    return ret == 0? 0 : -1;
}

int InspireHand::getStatus(int hand_idx) {
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int ret = 0;
    ret |= read_multi_param(hand_idx, STATUS_ADDR, false, joint_status_[hand_idx].data(), 6, 1);
    return ret == 0? 0 : -1;
}

int InspireHand::getCurrentLimit(int hand_idx){
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int ret = 0;
    ret |= read_multi_param(hand_idx, CURRENT_LIMIT_ADDR, false, joint_current_limit_[hand_idx].data(), 4, 2);
    ret |= read_multi_param(hand_idx, CURRENT_LIMIT_ADDR+8, false, joint_current_limit_[hand_idx].data()+4, 2, 2);
    return ret == 0? 0 : -1;
}

int InspireHand::getForceLimit(int hand_idx){
    if (hand_idx < 0 || hand_idx > 1) return -2;
    int ret = 0;
    ret |= read_multi_param(hand_idx, DEFAULT_FORCE_SET, false, joint_force_limit_[hand_idx].data(), 4, 2);
    ret |= read_multi_param(hand_idx, DEFAULT_FORCE_SET+8, false, joint_force_limit_[hand_idx].data()+4, 2, 2);
    return ret == 0? 0 : -1;
}

void InspireHand::decodeError(){
    for (int hand_idx=0; hand_idx < 2; hand_idx++){
        for(int joint_idx=0; joint_idx < 6; joint_idx++){
            char state = joint_error_[hand_idx][joint_idx];
            std::cout <<"Hand id: " << hand_idx <<", Joint id: "<< joint_idx <<", Error: ";
            if ((state & 0b00011111)== 0){
                std::cout << "None; ";
            }else{
                if ((state & 0b00000001)){
                    std::cout << "Linear actuator stalled; ";
                }
                if ((state & 0b00000010)){
                    std::cout << "Over temperature; ";
                }
                if ((state & 0b00000100)){
                    std::cout << "Overcurrent; ";
                }
                if ((state & 0b00001000)){
                    std::cout << "Abnormal motor; ";
                }
                if ((state & 0b00010000)){
                    std::cout << "Abnoraml Communication; ";
                }
            }
            std::cout << std::endl;
        }
    }
}

void InspireHand::decodeStatus(){
    for (int hand_idx=0; hand_idx < 2; hand_idx++){
        for(int joint_idx=0; joint_idx < 6; joint_idx++){
            char state = joint_error_[hand_idx][joint_idx];
            std::cout <<"Hand id: " << hand_idx <<", Joint id: "<< joint_idx <<", Status: ";
            if (state == 0){
                std::cout << "Releasing";
            }else if (state == 1){
                std::cout << "Closing";
            }else if (state == 2){
                std::cout << "Position reached";
            }else if (state == 3){
                std::cout << "Force reached";
            }else if (state == 4){
                std::cout << "Current protection reached";
            }else if (state == 5){
                std::cout << "Linear actuator stalled";
            }else if (state == 6){
                std::cout << "Adnormal motor";
            }
            std::cout << std::endl;
        }
    }
}
    
//------------------ 底层通信实现 ------------------//
int InspireHand::write_register(int hand_idx, unsigned reg_addr, bool is_wrist, int16_t value) {
    canable_CanMsg msg = {0};
    harmonic_CanMsg resp;
    
    msg.id = build_ext_id(hand_ids_[hand_idx], reg_addr, true, is_wrist);
    msg.isExtended = true;
    msg.dlc = 2;
    msg.data[0] = value & 0xFF;
    msg.data[1] = (value >> 8) & 0xFF;
    
    return send_can_frame(hand_idx, msg, resp);
}

int16_t InspireHand::read_register(int hand_idx, unsigned reg_addr, bool is_wrist) {
    canable_CanMsg msg = {0};
    harmonic_CanMsg resp;
    
    msg.id = build_ext_id(hand_ids_[hand_idx], reg_addr, false, is_wrist);
    msg.isExtended = true;
    msg.dlc = 1;
    msg.data[0] = 2; // 请求2字节
    
    if (send_can_frame(hand_idx, msg, resp) == CANABLE_SUCCESS) {
        return (resp.data[1] << 8) | resp.data[0];
    }
    return -1;
}

int InspireHand::write_multi_param(int hand_idx, unsigned reg_addr, bool is_wrist, const int16_t *values, int length, u_char size) {
    canable_CanMsg msg = {0};
    harmonic_CanMsg resp;
    
    msg.id = build_ext_id(hand_ids_[hand_idx], reg_addr, true, is_wrist);
    msg.isExtended = true;
    msg.dlc = length * size;

    if (size * length > 8){
        std::cout << "write_multi_param: size * length > 8 exceeded max. length of can data." << std::endl;
        return -1;
    }

    for (int i = 0; i < length; ++i) {
        for (u_char j = 0; j < size; ++j){
            msg.data[size*i+j] = (values[i] >> 8*j) & 0xFF;
        }
    }
    
    return send_can_frame(hand_idx, msg, resp);
}

int InspireHand::read_multi_param(int hand_idx, unsigned reg_addr, bool is_wrist, short* result, int length, u_char size) {
    canable_CanMsg msg = {0};
    harmonic_CanMsg resp;
    
    msg.id = build_ext_id(hand_ids_[hand_idx], reg_addr, false, is_wrist);
    msg.isExtended = true;
    msg.dlc = 1;
    msg.data[0] = length * size; // 请求的字节数
    
    if (send_can_frame(hand_idx, msg, resp) != CANABLE_SUCCESS) return CANABLE_FAILED_RECEIVEFAILED;
    
    if (size * length > 8){
        std::cout << "read_multi_param: size * length > 8 exceeded max. length of can data." << std::endl;
        return -1;
    }

    for (int i = 0; i < length; ++i) {
        result[i] = 0;
        for (u_char j = 0; j < size; ++j){
            result[i] |= resp.data[size*i+j] << 8*j;
        }
    }
    return CANABLE_SUCCESS;
}

//------------------ CAN协议工具函数 ------------------//
uint32_t InspireHand::build_ext_id(unsigned hand_id, unsigned reg_addr, bool is_write, bool is_wrist) {
    uint32_t rw_flag = is_wrist ? (is_write ? 5 : 4) : (is_write ? 1 : 0);
    return (rw_flag << 26) | ((reg_addr & 0xFFF) << 14) | (hand_id & 0x3FFF);
}

int InspireHand::send_can_frame(int hand_idx, const canable_CanMsg &msg, harmonic_CanMsg &response, int timeout) {
    response_flags_[hand_idx] = false;
    
    int ret = canable_writeMessage(can_channel_[hand_idx], &msg);
    if (ret != CANABLE_SUCCESS) return ret;

    // 单线程轮询等待
    auto start = std::chrono::steady_clock::now();
    while (true) {
        if (response_flags_[hand_idx]) {
            response = rx_msgs_[hand_idx];
            response_flags_[hand_idx] = false;
            return CANABLE_SUCCESS;
        }
        
        // 超时检查
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout) {
            return CANABLE_FAILED_RECEIVEFAILED;
        }
    }
}