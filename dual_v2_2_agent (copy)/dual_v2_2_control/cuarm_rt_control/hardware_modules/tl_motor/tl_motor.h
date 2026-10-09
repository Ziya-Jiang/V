/*
 * CANopen CiA 402 — expedited SDO only: pack tx_buffer, parse rx_buffer after bus I/O.
 * cxt_can_tl_motor / cxt_canfd_tl_motor own NMT, SDO exchange, and state machines.
 */

#ifndef TL_MOTOR_H
#define TL_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_POSITION = 8,
    MODE_VELOCITY = 9,
    MODE_TORQUE = 10,
} operation_mode_t;

typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERR_CAN_OPEN = -1,
    MOTOR_ERR_CAN_COMM = -2,
    MOTOR_ERR_MOTOR_FAULT = -3,
    MOTOR_ERR_NOT_ENABLED = -4,
    MOTOR_ERR_TIMEOUT = -5,
    MOTOR_ERR_INVALID_PARAM = -6,
    MOTOR_ERR_SDO_ABORT = -7,
} motor_error_t;

#define OD_CONTROL_WORD 0x6040
#define OD_STATUS_WORD 0x6041
#define OD_ERROR_CODE 0x603F
#define OD_OP_MODE 0x6060
#define OD_OP_MODE_DISP 0x6061
#define OD_ACTUAL_POS 0x6064
#define OD_ACTUAL_VEL 0x606C
#define OD_TARGET_TORQUE 0x6071
#define OD_ACTUAL_TORQUE 0x6077
#define OD_TARGET_POS 0x607A
#define OD_MAX_VEL 0x607F
#define OD_TARGET_VEL 0x60FF
#define OD_BRAKE_CTRL 0x200D

#define CMD_SHUTDOWN 0x0006
#define CMD_SWITCH_ON 0x0007
#define CMD_ENABLE_OP 0x000F
#define CMD_DISABLE_VOLT 0x0000
#define CMD_FAULT_RESET 0x0080

#define STATUS_READY_TO_SWITCH_ON (1 << 0)
#define STATUS_SWITCHED_ON (1 << 1)
#define STATUS_OPERATION_ENABLED (1 << 2)
#define STATUS_FAULT (1 << 3)
#define STATUS_QUICK_STOP (1 << 5)
#define STATUS_TARGET_REACHED (1 << 10)
/* 6041h 状态字 — 表 3-2 状态切换 */
#define TL_SW_INIT 0x0000
#define TL_SW_NO_FAULT 0x0260
#define TL_SW_READY 0x0221
#define TL_SW_WAIT_ENABLE 0x0233
#define TL_SW_RUNNING 0x0237
#define TL_SW_READY_FROM_RUN 0x0231
#define TL_SW_FAULT 0x021F

typedef enum {
    MOTOR_STATE_INIT = 0,
    MOTOR_STATE_NO_FAULT,
    MOTOR_STATE_READY_TO_SWITCH_ON,
    MOTOR_STATE_SWITCHED_ON,
    MOTOR_STATE_OPERATION_ENABLED,
    MOTOR_STATE_READY_FROM_RUN,
    MOTOR_STATE_FAULT,
    MOTOR_STATE_QUICK_STOP,
    MOTOR_STATE_UNKNOWN,
} motor_state_t;

typedef struct tl_motor {
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[8];
    uint8_t tx_len;
    uint8_t rx_len;
    uint8_t tx_id;
    uint8_t rx_id;
    float gear_ratio;
    int resolution;
    float torque_constant;
    operation_mode_t mode;
    uint16_t status_word;
    uint16_t error_code;
    float position;
    float velocity;
    float torque;
} tl_motor;

int tl_motor_init(tl_motor *motor, uint8_t node_id, operation_mode_t mode, float gear_ratio, int resolution, float torque_constant);    
void tl_motor_fault_reset(tl_motor *motor);
void tl_motor_set_shutdown(tl_motor *motor);
void tl_motor_set_switch_on(tl_motor *motor);
void tl_motor_set_enable(tl_motor *motor);
void tl_motor_set_disable(tl_motor *motor);
void tl_motor_set_torque(tl_motor *motor, float torque);
void tl_motor_set_velocity(tl_motor *motor, float velocity);
void tl_motor_set_position(tl_motor *motor, float position);
void tl_motor_set_max_velocity(tl_motor *motor, float max_velocity);

void tl_motor_get_position(tl_motor *motor);
void tl_motor_get_velocity(tl_motor *motor);
void tl_motor_get_torque(tl_motor *motor);
void tl_motor_get_error_code(tl_motor *motor);
void tl_motor_get_operation_mode(tl_motor *motor);
void tl_motor_get_status_word(tl_motor *motor);
void tl_motor_decoding(tl_motor *motor);
const char *tl_motor_state_to_string(uint16_t status_word);
//const char *tl_motor_error_to_string(motor_error_t error);

#ifdef __cplusplus
}
#endif

#endif
