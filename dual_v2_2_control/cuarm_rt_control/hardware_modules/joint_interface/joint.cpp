/**
 * @file joint.cpp
 * @brief `TlCanopenActuator` 与 `RmdCanActuator`：对接新版 `cxt_can` + `cxt_can_*_motor` 子模块。
 */

#include "joint.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

const double k_rad_to_deg = 180.0 / M_PI;
const double k_deg_to_rad = M_PI / 180.0;
constexpr uint16_t k_cia402_state_mask = 0x006Fu;
constexpr uint16_t k_cia402_ready_to_switch_on = 0x0021u;
constexpr uint16_t k_cia402_switched_on = 0x0023u;
constexpr uint16_t k_cia402_operation_enabled = 0x0027u;
int motor_err_to_int(motor_error_t e)
{
    return (e == MOTOR_OK) ? 0 : static_cast<int>(e);
}

bool tl_is_ready_to_switch_on(uint16_t status_word)
{
    return (status_word & k_cia402_state_mask) == k_cia402_ready_to_switch_on;
}

bool tl_is_switched_on(uint16_t status_word)
{
    return (status_word & k_cia402_state_mask) == k_cia402_switched_on;
}

bool tl_is_operation_enabled(uint16_t status_word)
{
    return (status_word & k_cia402_state_mask) == k_cia402_operation_enabled;
}

void ensure_single_node(cxt_can *bus, int can_channel)
{
    if (!bus || can_channel < 0 || can_channel >= CXT_CAN_MAX_CHANNEL) {
        return;
    }
    bus->node_num[can_channel] = 1;
}

double tl_raw_to_rad(const tl_motor &motor)
{
    const double scale = static_cast<double>(motor.gear_ratio) * static_cast<double>(motor.resolution);
    if (scale <= 0.0) {
        return 0.0;
    }
    return static_cast<double>(motor.position) / scale * k_deg_to_rad;
}

double tl_raw_vel_to_rad_s(const tl_motor &motor)
{
    const double scale = static_cast<double>(motor.gear_ratio) * static_cast<double>(motor.resolution);
    if (scale <= 0.0) {
        return 0.0;
    }
    return static_cast<double>(motor.velocity) / scale * k_deg_to_rad;
}

bool tl_velocity_is_near_zero(const tl_motor &motor)
{
    constexpr double k_stop_threshold_raw = 20.0;
    return std::fabs(static_cast<double>(motor.velocity)) <= k_stop_threshold_raw;
}

float tl_current_position_command_deg(const tl_motor &motor)
{
    return static_cast<float>(tl_raw_to_rad(motor) * 7.2 * k_rad_to_deg / 7.2);
}

int tl_wait_velocity_near_zero(cxt_can *bus, int can_channel, tl_motor *motor)
{
    // 必须**连续** N 次 |vel| < 阈值才认为停了。避免单次瞬时通过。
    constexpr int k_required_stable = 5;
    constexpr int k_max_step = 100;
    int stable_count = 0;
    for (int i = 0; i < k_max_step; ++i) {
        motor_error_t ret = cxt_can_tl_motor_get_velocity(bus, can_channel, motor);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        if (tl_velocity_is_near_zero(*motor)) {
            if (++stable_count >= k_required_stable) {
                return 0;
            }
        } else {
            stable_count = 0;
        }
        usleep(3000);
    }
    return -1;
}

int tl_report_fault(cxt_can *bus, int can_channel, tl_motor *motor, const char *stage)
{
    motor_error_t ret = cxt_can_tl_motor_get_error_code(bus, can_channel, motor);
    if (ret == MOTOR_OK) {
        
    } else {
        
    }
    return motor_err_to_int(MOTOR_ERR_MOTOR_FAULT);
}

int tl_prepare_stop_before_mode_switch(cxt_can *bus, int can_channel, tl_motor *motor, operation_mode_t current_mode)
{
    ensure_single_node(bus, can_channel);

    motor_error_t ret = MOTOR_OK;
    if (current_mode == MODE_VELOCITY) {
        float zero_vel[1] = {0.0f};
        ret = cxt_can_tl_motor_set_velocity(bus, can_channel, motor, zero_vel);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        return tl_wait_velocity_near_zero(bus, can_channel, motor);
    }

    if (current_mode == MODE_POSITION) {
        ret = cxt_can_tl_motor_get_position(bus, can_channel, motor);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }

        float hold_pos[1] = {tl_current_position_command_deg(*motor)};
        ret = cxt_can_tl_motor_set_position(bus, can_channel, motor, hold_pos);
        return motor_err_to_int(ret);
    }

    if (current_mode == MODE_TORQUE) {
        float zero_torque[1] = {0.0f};
        ret = cxt_can_tl_motor_set_torque(bus, can_channel, motor, zero_torque);
        return motor_err_to_int(ret);
    }

    return -1;
}

int tl_prepare_target_after_mode_switch(cxt_can *bus, int can_channel, tl_motor *motor, operation_mode_t mode)
{
    ensure_single_node(bus, can_channel);

    motor_error_t ret = MOTOR_OK;
    if (mode == MODE_POSITION) {
        // 切到 CSP 之前：必须**连续**读到稳定的位置（防止切模式瞬间位置还在变，
        // 导致 enable 后立刻 0x8611 跟随误差 fault）。
        int32_t last_pos = INT32_MIN;
        int stable_count = 0;
        constexpr int k_required_stable = 5;
        constexpr int k_max_step = 50;
        int32_t stable_pos = 0;
        for (int i = 0; i < k_max_step; ++i) {
            ret = cxt_can_tl_motor_get_position(bus, can_channel, motor);
            if (ret != MOTOR_OK) {
                return motor_err_to_int(ret);
            }
            const int32_t pos = motor->position;
            if (pos == last_pos) {
                if (++stable_count >= k_required_stable) {
                    stable_pos = pos;
                    break;
                }
            } else {
                last_pos = pos;
                stable_count = 0;
            }
            usleep(20000);
        }
        if (stable_count < k_required_stable) {
}

        // 把稳定位置写为 0x607A 目标 —— enable 后 drive 不会跑去别处
        // motor->position 此时已是稳定值（pulse），先转回 deg 再交给 set_position
        // （set_position 内部会 *gear*resolution 还原回 pulse）
        motor->position = stable_pos;
        float hold_pos[1] = {tl_current_position_command_deg(*motor)};
        ret = cxt_can_tl_motor_set_position(bus, can_channel, motor, hold_pos);
        return motor_err_to_int(ret);
    }

    if (mode == MODE_VELOCITY) {
        float zero_vel[1] = {0.0f};
        ret = cxt_can_tl_motor_set_velocity(bus, can_channel, motor, zero_vel);
        return motor_err_to_int(ret);
    }

    if (mode == MODE_TORQUE) {
        float zero_torque[1] = {0.0f};
        ret = cxt_can_tl_motor_set_torque(bus, can_channel, motor, zero_torque);
        return motor_err_to_int(ret);
    }

    return -1;
}

// 主动复位故障：发 0x6040=0x0080，并等待 status 退出 FAULT 态
// 返回 0 表示 fault 已清除；-1 表示仍处于故障（非可复位故障，需要断电）
int tl_reset_fault(cxt_can *bus, int can_channel, tl_motor *motor)
{
    motor_error_t ret = cxt_can_tl_motor_reset(bus, can_channel, motor);
    if (ret != MOTOR_OK) {
        return motor_err_to_int(ret);
    }
    constexpr int k_max_step = 20;
    for (int i = 0; i < k_max_step; ++i) {
        ret = cxt_can_tl_motor_get_status_word(bus, can_channel, motor);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        if (!(motor->status_word & STATUS_FAULT)) {
            return 0;
        }
        usleep(50000);
    }
    return -1;
}

// 把驱动从任意状态带到 "Switched On (0x0233)"，以便修改 0x6060。
// 完整状态机路径（按天链手册 v1.0.3 表 3-2）：
//   0x0000 (Init) / 0x0040 (Switch on disabled) / 0x0260 (No Fault)
//                    -- 0x0006 --> 0x0221 (Ready to Switch On)
//   0x0221 (Ready to Switch On)
//                    -- 0x0007 --> 0x0233 (Switched On)
//   0x0233 (Switched On)
//                    -- 0x000F --> 0x0237 (Operation Enabled)
//   0x0237 (Operation Enabled)
//                    -- 0x0007 --> 0x0233 (Switched On)
//   0x021F / 0x000F (Fault)
//                    -- 0x0080 --> 0x0260 (No Fault)
//
// 关键：冷启动时驱动是 0x0260 (No Fault)，对 0x0260 直接发 0x0007 无效，
// 必须先发 0x0006 → 0x0221，再发 0x0007 → 0x0233。
int tl_disable_operation_before_mode_switch(cxt_can *bus, int can_channel, tl_motor *motor)
{
    motor_error_t ret = cxt_can_tl_motor_get_status_word(bus, can_channel, motor);
    if (ret != MOTOR_OK) {
        return motor_err_to_int(ret);
    }

    // 1) FAULT 状态先发 0x0080 复位
    if (motor->status_word & STATUS_FAULT) {
        int rc = tl_reset_fault(bus, can_channel, motor);
        if (rc != 0) {
            return tl_report_fault(bus, can_channel, motor, "disable operation: fault not resettable");
        }
    }

    // 2) 已经在 Switched On (0x0233) 状态，直接返回
    if (tl_is_switched_on(motor->status_word)) {
        return 0;
    }

    // 3) 状态机推进到 Switched On (0x0233)
    //    注意：状态码提取用 & 0x006F (bit 0-3, 5, 6)。
    //    0x0260 (天链 No Fault) & 0x006F = 0x0060（不是 0x0260）。
    constexpr int k_max_step = 10;
    for (int i = 0; i < k_max_step; ++i) {
        const uint16_t state = motor->status_word & k_cia402_state_mask;

        // 已经在 Switched On
        if (state == k_cia402_switched_on) {
            return 0;
        }

        // 选发控制字：
        //   - Ready (0x0021) / Op Enabled (0x0027) → 0x0007 → Switched On
        //   - 其它（0x0000 / 0x0020 / 0x0040 / 0x0060 / 异常）→ 0x0006 → Ready
        if (state == k_cia402_ready_to_switch_on || state == k_cia402_operation_enabled) {
            ret = cxt_can_tl_motor_switch_on(bus, can_channel, motor);  // 0x0007
        } else {
            // 包含天链 No Fault (0x0260 & 0x006F = 0x0060)
            // 必须先发 0x0006 (Shutdown) 退到 Ready to Switch On
            ret = cxt_can_tl_motor_shutdown(bus, can_channel, motor);  // 0x0006
        }
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        ret = cxt_can_tl_motor_get_status_word(bus, can_channel, motor);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        if (motor->status_word & STATUS_FAULT) {
            return tl_report_fault(bus, can_channel, motor, "disable operation: fault during state transition");
        }
        usleep(100000);
    }
    return tl_report_fault(bus, can_channel, motor, "disable operation: timeout waiting switched on");
}

// (历史函数 tl_shutdown_until_ready 已移除 —— 模式切换现在直接在 Switched On 状态下进行。)

int tl_write_and_confirm_mode(cxt_can *bus, int can_channel, tl_motor *motor, operation_mode_t mode)
{
    motor_error_t ret = cxt_can_tl_motor_set_operation_mode(bus, can_channel, motor, &mode);
    if (ret != MOTOR_OK) {
        return motor_err_to_int(ret);
    }

    constexpr int k_max_step = 10;
    for (int i = 0; i < k_max_step; ++i) {
        ret = cxt_can_tl_motor_get_operation_mode_display(bus, can_channel, motor);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        if (motor->mode == mode) {
            return 0;
        }
        usleep(100000);
    }
    return -1;
}

int tl_enable_state_machine(cxt_can *bus, int can_channel, tl_motor *motor, operation_mode_t mode)
{
    ensure_single_node(bus, can_channel);

    motor_error_t ret = cxt_can_tl_motor_set_nmt(bus, can_channel, motor);
    if (ret != MOTOR_OK) {
        return motor_err_to_int(ret);
    }

    // CiA 402 顺序：必须先把状态退到 Switched On，才能修改 0x6060（configuration 参数）
    int rc = tl_disable_operation_before_mode_switch(bus, can_channel, motor);
    if (rc != 0) {
        return rc;
    }

    // 现在驱动在 Switched On 状态，可以安全写 0x6060
    rc = tl_write_and_confirm_mode(bus, can_channel, motor, mode);
    if (rc != 0) {
        return rc;
    }

    // 准备新模式的目标值（切 CSP 时 0x607A=当前位置；切 CSV 时 0x60FF=0）
    rc = tl_prepare_target_after_mode_switch(bus, can_channel, motor, mode);
    if (rc != 0) {
        return rc;
    }

    // 0x6040 = 0x000F，进入 Operation Enabled
    constexpr int k_max_step = 10;
    for (int i = 0; i < k_max_step; ++i) {
        ret = cxt_can_tl_motor_enable(bus, can_channel, motor);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        ret = cxt_can_tl_motor_get_status_word(bus, can_channel, motor);
        if (ret != MOTOR_OK) {
            return motor_err_to_int(ret);
        }
        if (motor->status_word & STATUS_FAULT) {
            return tl_report_fault(bus, can_channel, motor, "enable_state_machine: fault during enable");
        }
        if (tl_is_operation_enabled(motor->status_word)) {
            return 0;
        }
        if (i == k_max_step - 1) {
            return -1;
        }
        usleep(100000);
    }
    return -1;
}

// 内部子程序：发 0x000F 并轮询 status，最多 10 次
// 返回 0 = 已进 Operation Enabled
// 返回 >0 = 进了 fault（已 tl_report_fault 打印错误码）
// 返回 <0 = 通信/超时错误
static int tl_try_enable(cxt_can *bus, int can_channel, tl_motor *motor, const char *stage)
{
    constexpr int k_max_step = 10;
    motor_error_t mret = MOTOR_OK;
    for (int i = 0; i < k_max_step; ++i) {
        mret = cxt_can_tl_motor_enable(bus, can_channel, motor);  // 0x000F
        if (mret != MOTOR_OK) {
            return motor_err_to_int(mret);
        }
        mret = cxt_can_tl_motor_get_status_word(bus, can_channel, motor);
        if (mret != MOTOR_OK) {
            return motor_err_to_int(mret);
        }
        if (motor->status_word & STATUS_FAULT) {
            return tl_report_fault(bus, can_channel, motor, stage);
        }
        if (tl_is_operation_enabled(motor->status_word)) {
            return 0;
        }
        if (i == k_max_step - 1) {
            return -1;
        }
        usleep(100000);
    }
    return -1;
}

// 模式安全切换（带故障自恢复 + enable 重试）：
//   1) 把当前运动停下来（CSV->0, CSP->hold, CST->0）
//   2) 故障自复位 (0x6040=0x0080) 并退到 Switched On
//   3) 改写 0x6060，并在 Switched On 状态下准备好新模式的目标值
//   4) 0x6040=0x000F 进入 Operation Enabled
//   5) 如果 enable 触发 fault（最常见是 0x8611 跟随误差），
//      自动 fault reset + 重新写 0x6060 + 重新 enable 一次
int tl_switch_mode_safely(cxt_can *bus, int can_channel, tl_motor *motor, operation_mode_t current_mode,
                          operation_mode_t mode)
{
    if (current_mode == mode) {
        return 0;  // 模式相同无需切换
    }

    // 1) 减速到 0 / 锁定当前位置
    int ret = tl_prepare_stop_before_mode_switch(bus, can_channel, motor, current_mode);
    if (ret != 0) {
        return ret;
    }

    // 2) 故障自复位 + 退到 Switched On
    ret = tl_disable_operation_before_mode_switch(bus, can_channel, motor);
    if (ret != 0) {
        return ret;
    }

    // 3) 改写 0x6060
    ret = tl_write_and_confirm_mode(bus, can_channel, motor, mode);
    if (ret != 0) {
        return ret;
    }

    // 4) 在 Switched On 状态下预置新模式目标值
    //    CSP: 0x607A = 稳定位置（连续 5 次读到相同值）
    //    CSV: 0x60FF = 0
    //    CST: 0x6071 = 0
    ret = tl_prepare_target_after_mode_switch(bus, can_channel, motor, mode);
    if (ret != 0) {
        return ret;
    }

    // 5) 进入 Operation Enabled。最多 retry 2 次（首次 + 一次 fault-recover 后重试）
    for (int attempt = 0; attempt < 2; ++attempt) {
        const int rc = tl_try_enable(bus, can_channel, motor,
                                     attempt == 0 ? "switch_mode_safely: fault during enable (attempt 1)"
                                                  : "switch_mode_safely: fault during enable (attempt 2, after reset)");
        if (rc == 0) {
            return 0;  // 成功
        }
        if (rc < 0) {
            // 通信错误或超时 —— 不要 retry
            return rc;
        }
        // rc > 0 表示 drive 进了 fault
        if (attempt == 0) {
            // 第一次：尝试 fault reset 并重做 disable + write + target + enable
            
            if (tl_reset_fault(bus, can_channel, motor) != 0) {
                
                return tl_report_fault(bus, can_channel, motor, "fault not resettable after enable");
            }
            if (tl_disable_operation_before_mode_switch(bus, can_channel, motor) != 0) {
                return tl_report_fault(bus, can_channel, motor, "disable failed after reset");
            }
            if (tl_write_and_confirm_mode(bus, can_channel, motor, mode) != 0) {
                return tl_report_fault(bus, can_channel, motor, "rewrite mode failed after reset");
            }
            if (tl_prepare_target_after_mode_switch(bus, can_channel, motor, mode) != 0) {
                return tl_report_fault(bus, can_channel, motor, "rewrite target failed after reset");
            }
            // 下一轮 attempt 重新 enable
        } else {
            // 第二次还进 fault：放弃
            
            return rc;
        }
    }
    return -1;
}

} // namespace

/* ---------- TlCanopenActuator ---------- */

TlCanopenActuator::TlCanopenActuator(cxt_can *bus, int can_channel, uint8_t node_id, float gear_ratio,
                                     int resolution, float torque_constant)
    : bus_(bus), can_channel_(can_channel), node_id_(node_id)
{
    tl_motor_init(&motor_, node_id, MODE_POSITION, gear_ratio, resolution, torque_constant);
    if (bus_) {
        ensure_single_node(bus_, can_channel_);
    }
}

int TlCanopenActuator::request_iap_mode()
{
    return 0;
}

int TlCanopenActuator::set_absolute_position_rad(double pos_rad)
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    const float pos_deg = static_cast<float>(pos_rad * k_rad_to_deg) / 7.2;
    float pos_arr[1] = {pos_deg};
    const motor_error_t e = cxt_can_tl_motor_set_position(bus_, can_channel_, &motor_, pos_arr);
    control_success_ = (e == MOTOR_OK) ? 1 : 0;
    return motor_err_to_int(e);
}

int TlCanopenActuator::set_velocity(double vel_rad_s)
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    const float vel_deg_s = static_cast<float>(vel_rad_s * k_rad_to_deg) / 7.2f * 10;
    float vel_arr[1] = {vel_deg_s};
    const motor_error_t e = cxt_can_tl_motor_set_velocity(bus_, can_channel_, &motor_, vel_arr);
    control_success_ = (e == MOTOR_OK) ? 1 : 0;
    return motor_err_to_int(e);
}

int TlCanopenActuator::set_current(double cur_A)
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    const float cur_f = static_cast<float>(cur_A) / 31.0f * 1000.0f / 100.0f / 0.001f;  // A -> mA
    float tor_arr[1] = {cur_f};
    const motor_error_t e = cxt_can_tl_motor_set_torque(bus_, can_channel_, &motor_, tor_arr);
    control_success_ = (e == MOTOR_OK) ? 1 : 0;
    return motor_err_to_int(e);
}

int TlCanopenActuator::read_status()
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);

    motor_error_t e = cxt_can_tl_motor_get_position(bus_, can_channel_, &motor_);
    if (e != MOTOR_OK) {
        control_success_ = 0;
        return motor_err_to_int(e);
    }
    pos_rad_ = tl_raw_to_rad(motor_)  * 7.2;

    e = cxt_can_tl_motor_get_velocity(bus_, can_channel_, &motor_) ;
    if (e != MOTOR_OK) {
        control_success_ = 0;
        return motor_err_to_int(e);
    }
    vel_rad_s_ = tl_raw_vel_to_rad_s(motor_) * 7.2 / 10;

    e = cxt_can_tl_motor_get_torque(bus_, can_channel_, &motor_);
    if (e != MOTOR_OK) {
        control_success_ = 0;
        return motor_err_to_int(e);
    }
    cur_A_ = static_cast<double>(motor_.torque) / 1000.0f * 31.0f;
    control_success_ = 1;
    return 0;
}

int TlCanopenActuator::enable()
{
    if (!bus_) {
        return -1;
    }
    const int r = tl_enable_state_machine(bus_, can_channel_, &motor_, current_mode_);
    control_success_ = (r == 0) ? 1 : 0;
    return r;
}

int TlCanopenActuator::disable()
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    const motor_error_t e = cxt_can_tl_motor_disable(bus_, can_channel_, &motor_);
    control_success_ = (e == MOTOR_OK) ? 1 : 0;
    return motor_err_to_int(e);
}

int TlCanopenActuator::clear_error()
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    const motor_error_t e = cxt_can_tl_motor_reset(bus_, can_channel_, &motor_);
    control_success_ = (e == MOTOR_OK) ? 1 : 0;
    return motor_err_to_int(e);
}

int TlCanopenActuator::select_mode(JointControlMode mode)
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    operation_mode_t om = MODE_POSITION;
    switch (static_cast<int>(mode)) {
    case static_cast<int>(JointControlMode::Current):
        om = MODE_TORQUE;
        break;
    case static_cast<int>(JointControlMode::Velocity):
        om = MODE_VELOCITY;
        break;
    case static_cast<int>(JointControlMode::Position):
        om = MODE_POSITION;
        break;
    default:
        return -1;
    }
    if (current_mode_ == om) {
        control_success_ = 1;
        return 0;
    }

    const int r = tl_switch_mode_safely(bus_, can_channel_, &motor_, current_mode_, om);
    if (r == 0) {
        current_mode_ = om;
    }
    control_success_ = (r == 0) ? 1 : 0;
    return r;
}

void TlCanopenActuator::prepare_command_motor(JointControlMode mode, double actuator_value, tl_motor *out) const
{
    if (!out) return;
    *out = motor_;
    switch (static_cast<int>(mode)) {
    case static_cast<int>(JointControlMode::Current): {
        const float cur_f = static_cast<float>(actuator_value) / 31.0f * 1000.0f / 100.0f / 0.001f;
        tl_motor_set_torque(out, cur_f);
        break;
    }
    case static_cast<int>(JointControlMode::Velocity): {
        const float vel_deg_s = static_cast<float>(actuator_value * k_rad_to_deg) / 7.2f * 10;
        tl_motor_set_velocity(out, vel_deg_s);
        break;
    }
    case static_cast<int>(JointControlMode::Position): {
        const float pos_deg = static_cast<float>(actuator_value * k_rad_to_deg) / 7.2f * 10;
        tl_motor_set_position(out, pos_deg);
        break;
    }
    default:
        break;
    }
}

void TlCanopenActuator::apply_feedback(const tl_motor &updated)
{
    motor_ = updated;
    pos_rad_   = tl_raw_to_rad(motor_) * 7.2;
    vel_rad_s_ = tl_raw_vel_to_rad_s(motor_) * 7.2 / 10;
    cur_A_     = static_cast<double>(motor_.torque) / 1000.0f * 31.0f;
    control_success_ = 1;
}

int TlCanopenActuator::decode_feedback(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    (void)can_id;
    (void)data;
    (void)dlc;
    return 0;
}

/* ---------- RmdCanActuator ---------- */

RmdCanActuator::RmdCanActuator(cxt_can *bus, int can_channel, uint32_t node_id, MOTOR_TYPE type, float gear_ratio,
                               int resolution, bool has_brake)
    : bus_(bus), can_channel_(can_channel), has_brake_(has_brake)
{
    rmd_motor_init(&motor_, type, static_cast<int>(node_id), gear_ratio, resolution);
    if (bus_) {
        ensure_single_node(bus_, can_channel_);
    }
}

int RmdCanActuator::set_absolute_position_rad(double pos_rad)
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    const float pos_deg = static_cast<float>(pos_rad * k_rad_to_deg);
    float pos_arr[1] = {pos_deg};
    control_success_ = 0;
    const int r = cxt_can_rmd_motor_set_position(bus_, can_channel_, &motor_, pos_arr);
    if (r == 0) {
        apply_command_feedback(motor_);
    }
    return r;
}

int RmdCanActuator::set_velocity(double vel_rad_s)
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    const float vel_deg_s = static_cast<float>(vel_rad_s * k_rad_to_deg);
    float vel_arr[1] = {vel_deg_s};
    control_success_ = 0;
    const int r = cxt_can_rmd_motor_set_velocity(bus_, can_channel_, &motor_, vel_arr);
    if (r == 0) {
        apply_command_feedback(motor_);
    }
    return r;
}

int RmdCanActuator::set_current(double cur_A)
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    // const float cur_f = static_cast<float>(cur_A) / 2.5f;
    const float cur_f = static_cast<float>(cur_A);
    float tor_arr[1] = {cur_f};
    control_success_ = 0;
    const int r = cxt_can_rmd_motor_set_torque(bus_, can_channel_, &motor_, tor_arr);
    if (r == 0) {
        apply_command_feedback(motor_);
    }
    return r;
}

int RmdCanActuator::read_absolute_position()
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    rmd_motor_get_position(&motor_);
    control_success_ = 0;
    const int r = cxt_can_rmd_motor_exchange(bus_, can_channel_, &motor_);
    if (r != 0) {
        return -1;
    }
    pos_rad_ = static_cast<double>(motor_.position) * k_deg_to_rad;
    control_success_ = 1;
    return 0;
}

int RmdCanActuator::read_status()
{
    return read_absolute_position();
}

int RmdCanActuator::prepare_command_motor(JointControlMode mode, double actuator_value, rmd_motor *out_motor) const
{
    if (!out_motor) {
        return -1;
    }
    *out_motor = motor_;
    switch (static_cast<int>(mode)) {
    case static_cast<int>(JointControlMode::Current):
        rmd_motor_set_torque(out_motor, static_cast<float>(actuator_value));
        return 0;
    case static_cast<int>(JointControlMode::Velocity):
        rmd_motor_set_velocity(out_motor, static_cast<float>(actuator_value * k_rad_to_deg));
        return 0;
    case static_cast<int>(JointControlMode::Position):
        rmd_motor_set_position(out_motor, static_cast<float>(actuator_value * k_rad_to_deg));
        return 0;
    default:
        return -1;
    }
}

void RmdCanActuator::apply_command_feedback(const rmd_motor &updated_motor)
{
    motor_ = updated_motor;
    vel_rad_s_ = static_cast<double>(motor_.velocity) * k_deg_to_rad;
    cur_A_ = static_cast<double>(motor_.torque);
    // cur_A_ = static_cast<double>(motor_.torque) * 2.5f;
    control_success_ = 1;
}

int RmdCanActuator::enable()
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    int r;
    if (!has_brake_) {
        /* 无抱闸电机：disable() 时发的是 0xA4 位置锁定而非 0x80 shutdown，
         * 电机未掉使能，不需要 resume(0x88)。直接用 0xA4 以当前位置接管控制权，
         * 避免 resume 清除位置锁定导致关节瞬时失力下坠。 */
        r = read_absolute_position();
        if (r != 0) {
            control_success_ = 0;
            return r;
        }
        rmd_motor_set_position(&motor_, motor_.position);
        r = cxt_can_rmd_motor_exchange(bus_, can_channel_, &motor_);
    } else {
        /* 有物理抱闸：原有 resume + brake 流程 */
        r = cxt_can_rmd_motor_enable(bus_, can_channel_, &motor_);
    }
    control_success_ = (r == 0) ? 1 : 0;
    return r;
}

int RmdCanActuator::disable()
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    int r;
    if (has_brake_) {
        /* 有物理抱闸：正常 shutdown，抱闸机械锁死 */
        r = cxt_can_rmd_motor_shutdown(bus_, can_channel_, &motor_);
    } else {
        /* 无抱闸：发 0xA4 锁定当前位置，驱动器内部位置环保持 */
        r = lock_position();
    }
    control_success_ = (r == 0) ? 1 : 0;
    return r;
}

int RmdCanActuator::lock_position()
{
    if (!bus_) {
        return -1;
    }
    ensure_single_node(bus_, can_channel_);
    /* 先读取当前多圈绝对位置，保证 motor_.position 是最新的 */
    int r = read_absolute_position();
    if (r != 0) {
        return r;
    }
    /* 发 0xA4 + maxSpeed=0，锁死当前位置 */
    r = cxt_can_rmd_motor_lock_position(bus_, can_channel_, &motor_);
    control_success_ = (r == 0) ? 1 : 0;
    return r;
}

int RmdCanActuator::select_mode(JointControlMode mode)
{
    // RMD motors determine operating mode by CAN command type:
    //   0xA1 = current(torque), 0xA2 = velocity, 0xA4 = position.
    // There is no separate "mode register" — the mode is implicit in
    // whichever command is sent next.  Do NOT send a zero-current or
    // zero-velocity frame here; that would overwrite the position hold
    // from enable() and cause the joint to drop before the first real
    // control command arrives.
    (void)mode;
    return 0;
}

int RmdCanActuator::decode_feedback(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    (void)dlc;
    const uint32_t expect = static_cast<uint32_t>(motor_.motor_id) + 0x140u;
    if ((can_id & 0x7FFu) != (expect & 0x7FFu)) {
        return 0;
    }
    uint8_t buf[8];
    memcpy(buf, data, 8);
    rmd_motor_decoding(&motor_, buf);
    if (buf[0] == 0x92) {
        pos_rad_ = static_cast<double>(motor_.position) * k_deg_to_rad;
        control_success_ = 1;
        return 0;
    }
    apply_command_feedback(motor_);
    return 0;
}
