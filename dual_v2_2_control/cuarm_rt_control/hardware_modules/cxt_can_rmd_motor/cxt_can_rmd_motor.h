/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : cxt_can_rmd_motor.h                                        *
 * Author      : CHEN Wei, Simon Tam, Black                                 *
 * Version     : 4.0.0                                                      *
 * Date        : 2026-05-06                                                 *
 * Description : RMD motor helpers for Chuangxin classic CAN (ControlCAN).  *
 * Others      : None                                                       *
 * History     : 2026-05-06 Align API with cxt_canfd_rmd_motor + rmd_motor.  *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#ifndef CXT_CAN_RMD_MOTOR_H
#define CXT_CAN_RMD_MOTOR_H

#include "rmd_motor.h"
#include "cxt_can.h"

#ifdef __cplusplus
extern "C" {
#endif

int cxt_can_rmd_motor_pack(cxt_can *bus, int node_idx, rmd_motor *motor);
int cxt_can_rmd_motor_unpack(cxt_can *bus, int node_idx, rmd_motor *motor);
int cxt_can_rmd_motor_exchange(cxt_can *bus, int can_idx, rmd_motor *motor);
int cxt_can_rmd_motor_enable(cxt_can *bus, int can_idx, rmd_motor *motor);
int cxt_can_rmd_motor_disable(cxt_can *bus, int can_idx, rmd_motor *motor);
int cxt_can_rmd_motor_set_torque(cxt_can *bus, int can_idx, rmd_motor *motor, float tor[]);
int cxt_can_rmd_motor_set_velocity(cxt_can *bus, int can_idx, rmd_motor *motor, float vel[]);
int cxt_can_rmd_motor_set_position(cxt_can *bus, int can_idx, rmd_motor *motor, float pos[]);
int cxt_can_rmd_motor_shutdown(cxt_can *bus, int can_idx, rmd_motor *motor);
int cxt_can_rmd_motor_lock_position(cxt_can *bus, int can_idx, rmd_motor *motor);
int cxt_can_rmd_motor_pause(cxt_can *bus, int can_idx, rmd_motor *motor);
int cxt_can_rmd_motor_resume(cxt_can *bus, int can_idx, rmd_motor *motor);

#ifdef __cplusplus
}
#endif

#endif
