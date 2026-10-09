#include "tl_motor.h"
#include <string.h>

// ========== SDO 写函数 ==========
void sdo_write_u8(tl_motor *motor, uint16_t index, uint8_t subindex, uint8_t value)
{
    motor->tx_len = 8;
    motor->tx_buffer[0] = 0x2F;
    motor->tx_buffer[1] = (uint8_t)(index & 0xFF);
    motor->tx_buffer[2] = (uint8_t)(index >> 8);
    motor->tx_buffer[3] = subindex;
    motor->tx_buffer[4] = value;
    motor->tx_buffer[5] = 0;
    motor->tx_buffer[6] = 0;
    motor->tx_buffer[7] = 0;
}

void sdo_write_u16(tl_motor *motor, uint16_t index, uint8_t subindex, uint16_t value)
{
    motor->tx_len = 8;
    motor->tx_buffer[0] = 0x2B;
    motor->tx_buffer[1] = (uint8_t)(index & 0xFF);
    motor->tx_buffer[2] = (uint8_t)(index >> 8);
    motor->tx_buffer[3] = subindex;
    motor->tx_buffer[4] = (uint8_t)(value & 0xFF);
    motor->tx_buffer[5] = (uint8_t)(value >> 8);
    motor->tx_buffer[6] = 0;
    motor->tx_buffer[7] = 0;
}

void sdo_write_u32(tl_motor *motor, uint16_t index, uint8_t subindex, uint32_t value)
{
    motor->tx_len = 8;
    motor->tx_buffer[0] = 0x23;
    motor->tx_buffer[1] = (uint8_t)(index & 0xFF);
    motor->tx_buffer[2] = (uint8_t)(index >> 8);
    motor->tx_buffer[3] = subindex;
    motor->tx_buffer[4] = (uint8_t)(value & 0xFF);
    motor->tx_buffer[5] = (uint8_t)((value >> 8) & 0xFF);
    motor->tx_buffer[6] = (uint8_t)((value >> 16) & 0xFF);
    motor->tx_buffer[7] = (uint8_t)((value >> 24) & 0xFF);
}

void sdo_write_s16(tl_motor *motor, uint16_t index, uint8_t subindex, int16_t value)
{
    sdo_write_u16(motor, index, subindex, (uint16_t)value);
}

void sdo_write_s32(tl_motor *motor, uint16_t index, uint8_t subindex, int32_t value)
{
    sdo_write_u32(motor, index, subindex, (uint32_t)value);
}

// ========== SDO 读函数 ==========
void sdo_read(tl_motor *motor, uint16_t index, uint8_t subindex)
{
    motor->tx_len = 8;
    motor->tx_buffer[0] = 0x40;
    motor->tx_buffer[1] = (uint8_t)(index & 0xFF);
    motor->tx_buffer[2] = (uint8_t)(index >> 8);
    motor->tx_buffer[3] = subindex;
    motor->tx_buffer[4] = 0;
    motor->tx_buffer[5] = 0;
    motor->tx_buffer[6] = 0;
    motor->tx_buffer[7] = 0;
}

// ========== 电机初始化与控制 ==========
int tl_motor_init(tl_motor* motor, uint8_t id, operation_mode_t mode, float gear_ratio, int resolution, float torque_constant)
{
    motor->tx_id = id;     
    motor->mode = mode;     
    motor->gear_ratio = gear_ratio;
    motor->resolution = resolution;
    motor->torque_constant = torque_constant;
    motor->tx_len = 0;
    return 0;
}

void tl_motor_fault_reset(tl_motor *motor)
{
    sdo_write_u16(motor, OD_CONTROL_WORD, 0, CMD_FAULT_RESET);
}

void tl_motor_set_shutdown(tl_motor *motor)
{
    sdo_write_u16(motor, OD_CONTROL_WORD, 0, CMD_SHUTDOWN);
}

void tl_motor_set_switch_on(tl_motor *motor)
{
    sdo_write_u16(motor, OD_CONTROL_WORD, 0, CMD_SWITCH_ON);
}

void tl_motor_set_enable(tl_motor *motor)
{
    sdo_write_u16(motor, OD_CONTROL_WORD, 0, CMD_ENABLE_OP);
}

void tl_motor_set_disable(tl_motor *motor)
{
    sdo_write_u16(motor, OD_CONTROL_WORD, 0, CMD_DISABLE_VOLT);
}

void tl_motor_set_torque(tl_motor *motor, float torque)
{
    sdo_write_u16(motor, OD_TARGET_TORQUE, 0, (uint16_t)(torque * motor->gear_ratio * motor->torque_constant));
}

void tl_motor_set_velocity(tl_motor *motor, float velocity)
{
    sdo_write_u32(motor, OD_TARGET_VEL, 0, (uint32_t)(velocity * motor->gear_ratio * motor->resolution));
}

void tl_motor_set_max_velocity(tl_motor *motor, float max_velocity)
{
    sdo_write_u32(motor, OD_MAX_VEL, 0, (uint32_t)(max_velocity * motor->gear_ratio * motor->resolution));
}

void tl_motor_set_position(tl_motor *motor, float position)
{
    sdo_write_s32(motor, OD_TARGET_POS, 0, (int32_t)(position * motor->gear_ratio * motor->resolution));
}

void tl_motor_get_position(tl_motor *motor)
{
    sdo_read(motor, OD_ACTUAL_POS, 0);
}

void tl_motor_get_velocity(tl_motor *motor)
{
    sdo_read(motor, OD_ACTUAL_VEL, 0);
}

void tl_motor_get_torque(tl_motor *motor)
{
    sdo_read(motor, OD_ACTUAL_TORQUE, 0);
}

void tl_motor_get_error_code(tl_motor *motor) 
{
    sdo_read(motor, OD_ERROR_CODE, 0);
}

void tl_motor_get_operation_mode(tl_motor *motor)
{
    sdo_read(motor, OD_OP_MODE, 0);
}

void tl_motor_get_operation_mode_display(tl_motor *motor)
{
    sdo_read(motor, OD_OP_MODE_DISP, 0);
}

void tl_motor_set_operation_mode(tl_motor *motor, operation_mode_t mode)
{
    sdo_write_u8(motor, OD_OP_MODE, 0, (uint8_t)mode);
}

void tl_motor_get_status_word(tl_motor *motor)
{
    sdo_read(motor, OD_STATUS_WORD, 0);
}

void tl_motor_get_quick_stop_option(tl_motor *motor)
{
    sdo_read(motor, 0x605A, 0);
}

void tl_motor_set_quick_stop_option(tl_motor *motor, uint32_t value)
{
    sdo_write_u32(motor, 0x605A, 0, value);
}

void tl_motor_get_brake_control(tl_motor *motor)
{
    sdo_read(motor, OD_BRAKE_CTRL, 0);
}

void tl_motor_set_brake_control(tl_motor *motor, uint8_t value)
{
    sdo_write_u8(motor, OD_BRAKE_CTRL, 0, value);
}

const char *tl_motor_state_to_string(uint16_t state) 
{
    switch (state) {
        case TL_SW_INIT:
            return "Initialization";
        case TL_SW_NO_FAULT:
            return "No Fault";
        case TL_SW_READY:
            return "Ready to Switch On";
        case TL_SW_WAIT_ENABLE:
            return "Switched On";
        case TL_SW_RUNNING:
            return "Operation Enabled";
        case TL_SW_READY_FROM_RUN:
            return "Ready from Run";
        case TL_SW_FAULT:
            return "Fault";
        default:
            if (state & STATUS_FAULT)
                return "Fault";
            if (state & STATUS_QUICK_STOP)
                return "Quick Stop";
            return "Unknown State";
    }
}

void tl_motor_decoding(tl_motor *motor)
{
    uint16_t index = (uint16_t)motor->rx_buffer[1] | ((uint16_t)motor->rx_buffer[2] << 8);
    // if (index == OD_STATUS_WORD) {
    //     printf("Decoding ID: %04X, Index: %04X, Data: ", motor->rx_id, index);
    //     for (int i = 0; i < motor->rx_len; ++i) {
    //         printf(" %02X", motor->rx_buffer[i]);
    //     }
    //     printf("\n");
    // }
    switch (index) {
    case OD_STATUS_WORD:
        motor->status_word = (uint16_t)(motor->rx_buffer[4] | ((uint16_t)motor->rx_buffer[5] << 8));
        break;
    case OD_ERROR_CODE:
        motor->error_code = (uint16_t)(motor->rx_buffer[4] | ((uint16_t)motor->rx_buffer[5] << 8));
        break;
    case OD_OP_MODE:
    case OD_OP_MODE_DISP:
        motor->mode = (operation_mode_t)motor->rx_buffer[4];
        break;
    case OD_ACTUAL_POS:
        motor->position = (int32_t)(motor->rx_buffer[4] | ((uint32_t)motor->rx_buffer[5] << 8) | ((uint32_t)motor->rx_buffer[6] << 16) | ((uint32_t)motor->rx_buffer[7] << 24));
        break;
    case OD_ACTUAL_VEL:
        motor->velocity = (int32_t)(motor->rx_buffer[4] | ((uint32_t)motor->rx_buffer[5] << 8) | ((uint32_t)motor->rx_buffer[6] << 16) | ((uint32_t)motor->rx_buffer[7] << 24));
        break;
    case OD_ACTUAL_TORQUE:
        motor->torque = (int16_t)(motor->rx_buffer[4] | ((int16_t)motor->rx_buffer[5] << 8));
        break;
    case 0x605A:  // Quick Stop Option Code
    case 0x200D:  // Brake Control
        motor->status_word = (uint16_t)(motor->rx_buffer[4] | ((uint16_t)motor->rx_buffer[5] << 8));
        break;
    default:
        break;
    }
}
