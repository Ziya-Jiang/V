/********** ********** ********** ********** ********** ********** **********
 * TL motor — classic CAN (CiA402) on Chuangxin ControlCAN.
 * ../tl_motor: frame pack/parse + constants; this file: CAN TX/RX + CiA402.
 ********** ********** ********** ********** ********** ********** **********/

#ifndef CXT_CAN_TL_MOTOR_H
#define CXT_CAN_TL_MOTOR_H

#include <stdint.h>
#include "tl_motor.h"
#include "cxt_can.h"

void tl_motor_set_operation_mode(tl_motor *motor, operation_mode_t mode);
void tl_motor_get_operation_mode_display(tl_motor *motor);

#ifdef __cplusplus
extern "C" {
#endif

motor_error_t cxt_can_tl_motor_reset(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_shutdown(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_reset(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_switch_on(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_enable(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_disable(cxt_can *bus, int can_idx, tl_motor *motor);

motor_error_t cxt_can_tl_motor_set_nmt(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_set_operation_mode(cxt_can *bus, int can_idx, tl_motor *motor, operation_mode_t mode[]);
motor_error_t cxt_can_tl_motor_set_position(cxt_can *bus, int can_idx, tl_motor *motor, float position[]);
motor_error_t cxt_can_tl_motor_set_velocity(cxt_can *bus, int can_idx, tl_motor *motor, float velocity[]);
motor_error_t cxt_can_tl_motor_set_torque(cxt_can *bus, int can_idx, tl_motor *motor, float torque[]);
motor_error_t cxt_can_tl_motor_set_max_velocity(cxt_can *bus, int can_idx, tl_motor *motor, float max_velocity[]);

motor_error_t cxt_can_tl_motor_get_status_word(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_get_operation_mode(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_get_operation_mode_display(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_get_position(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_get_velocity(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_get_torque(cxt_can *bus, int can_idx, tl_motor *motor);
motor_error_t cxt_can_tl_motor_get_error_code(cxt_can *bus, int can_idx, tl_motor *motor);

/* 底层 SDO exchange：pack → send → receive → unpack，支持多节点批量 */
int cxt_can_tl_motor_exchange(cxt_can *bus, int chn_idx, tl_motor *motor);

#ifdef __cplusplus
}
#endif

#endif
