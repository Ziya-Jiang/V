#ifndef HKCLR_JOINT_H
#define HKCLR_JOINT_H

/**
 * @file joint.h
 * @brief `JointActuatorBase` 的具体实现：TL（CANopen）与 RMD 两类执行器。
 *
 * 依赖已由上层打开的 `cxt_can`（经典 CAN）；本头文件不拥有总线生命周期。
 * 与 `joint_interface.h` 中的单位与返回值约定一致。
 */

#include "joint_interface.h"
#include "cxt_can.h"
#include "cxt_can_rmd_motor.h"
#include "cxt_can_tl_motor.h"
#include "rmd_motor.h"
#include "tl_motor.h"
#include <cstdint>

/**
 * TL 系列关节：CANopen SDO / CiA402 状态机经 `cxt_can_tl_motor` 访问。
 *
 * @param bus              已 `cxt_can_init` 的总线对象；本类不取得所有权。
 * @param can_channel      `cxt_can` 逻辑通道下标（0 .. can_num-1），非 VCI 物理端口号。
 * @param node_id          CANopen 节点 ID（1–127）。
 * @param gear_ratio       减速比，传入 `tl_motor_init`。
 * @param resolution       编码器分辨率，传入 `tl_motor_init`。
 * @param torque_constant  力矩常数，传入 `tl_motor_init`。
 *
 * 位置/速度在 TL 驱动侧以度（deg）与度每秒（deg/s）交互；对外仍用 rad / rad/s。
 */
class TlCanopenActuator : public JointActuatorBase {
public:
    explicit TlCanopenActuator(cxt_can *bus, int can_channel, uint8_t node_id, float gear_ratio = 100.0f,
                                 int resolution = 100, float torque_constant = 0.001f);

    int request_iap_mode() override;
    int set_absolute_position_rad(double pos_rad) override;
    int set_velocity(double vel_rad_s) override;
    int set_current(double cur_A) override;
    int read_status() override;
    int enable() override;
    int disable() override;
    int clear_error() override;
    int select_mode(JointControlMode mode) override;
    int decode_feedback(uint32_t can_id, const uint8_t *data, uint8_t dlc) override;

    double get_position() const override { return pos_rad_; }
    double get_velocity() const override { return vel_rad_s_; }
    double get_current() const override { return cur_A_; }
    int is_control_success() const override { return control_success_; }
    JointKind joint_kind() const override { return JointKind::TlCanopen; }
    uint32_t can_node_id() const override { return node_id_; }

    /* Batch support: 供 LiteArmV2 批量 SDO exchange 使用 */
    int can_channel() const { return can_channel_; }
    void prepare_command_motor(JointControlMode mode, double actuator_value, tl_motor *out) const;
    void apply_feedback(const tl_motor &updated);
    bool accepts_can_id(uint32_t can_id) const override
    {
        const uint32_t id11 = can_id & 0x7FFu;
        return (id11 & 0x7Fu) == (can_node_id() & 0x7Fu);
    }

private:
    cxt_can *bus_{nullptr};
    int can_channel_{0};
    uint8_t node_id_{0};
    tl_motor motor_{};
    operation_mode_t current_mode_{MODE_POSITION};
    double pos_rad_{0};
    double vel_rad_s_{0};
    double cur_A_{0};
    int control_success_{0};
};

/**
 * RMD 系列关节：`rmd_motor` 组帧 + `cxt_can_rmd_motor_*` 收发。
 *
 * @param bus           已初始化的 `cxt_can`。
 * @param can_channel   物理 CAN 通道号。
 * @param node_id       RMD 设备 ID。
 * @param type          `rmd_motor` 解码与标定用型号。
 * @param gear_ratio    减速比，传入 `rmd_motor_init`。
 * @param resolution    编码器分辨率，传入 `rmd_motor_init`。
 */
class RmdCanActuator : public JointActuatorBase {
public:
    explicit RmdCanActuator(cxt_can *bus, int can_channel, uint32_t node_id, MOTOR_TYPE type = RMD_V4,
                            float gear_ratio = 1.0f, int resolution = 8192,
                            bool has_brake = true);

    int request_iap_mode() override { return 0; }
    int set_absolute_position_rad(double pos_rad) override;
    int set_velocity(double vel_rad_s) override;
    int set_current(double cur_A) override;
    int read_status() override;
    /** 0x92 读多圈绝对位置；不更新 vel/tor，避免覆盖 A1/A2 应答。 */
    int read_absolute_position();

    int enable() override;
    int disable() override;
    int lock_position();
    int clear_error() override { return 0; }
    int select_mode(JointControlMode mode) override;
    int decode_feedback(uint32_t can_id, const uint8_t *data, uint8_t dlc) override;
    int prepare_command_motor(JointControlMode mode, double actuator_value, rmd_motor *out_motor) const;
    void apply_command_feedback(const rmd_motor &updated_motor);
    int can_channel() const { return can_channel_; }

    double get_position() const override { return pos_rad_; }
    double get_velocity() const override { return vel_rad_s_; }
    double get_current() const override { return cur_A_; }
    int is_control_success() const override { return control_success_; }
    JointKind joint_kind() const override { return JointKind::Rmd; }
    uint32_t can_node_id() const override { return static_cast<uint32_t>(motor_.motor_id); }
    bool accepts_can_id(uint32_t can_id) const override
    {
        const uint32_t id11 = can_id & 0x7FFu;
        const uint32_t nid = can_node_id() & 0xFFu;
        return id11 >= 0x140u && id11 <= 0x23Fu && (id11 - 0x140u) == nid;
    }

private:
    cxt_can *bus_{nullptr};
    int can_channel_{0};
    rmd_motor motor_{};
    double pos_rad_{0};
    double vel_rad_s_{0};
    double cur_A_{0};
    int control_success_{0};
    bool has_brake_{true};
};

#endif
