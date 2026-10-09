/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2023 CURI & HKCLR                                   *
 * File name   : rmd_motor.h                                               *
 * Author      : CHEN Wei, Simon Tam, Black                                 *
 * Version     : 2.0.0                                                      *
 * Date        : 2023-01-01                                                 *
 * Description : Motor data structure.                                      *
 * Others      : None                                                       *
 * History     : 2022-04-05 1st version.                                    *
 *             : 2023-01-01 2nd version. Add motor type and redefine the    *
 *             :            data type.                                      *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <stdint.h>

#ifndef MOTOR_TYPE_DEFINE
#define MOTOR_TYPE_DEFINE
typedef enum {
    RMD_V1 = 1,
    RMD_V3 = 2,
    RMD_V4 = 3,
    RMD_V4_1 = 4,
    RMD_V999 = 999,
}MOTOR_TYPE;
#endif

#ifndef RMD_MOTOR_DEFINE
#define RMD_MOTOR_DEFINE
typedef struct {
    int lock;
    int cmd_id, motor_id, rx_len;
    unsigned char tx_buffer[8], rx_buffer[8];
    int resolution;
    float torque_constant;
    float gear_ratio;
    float position, velocity, torque, current;
    float raw_position;
    float last_position;
    float theta_prev; /* single-turn joint angle (deg), for unwrap */
    int circle, has_first_data;
    MOTOR_TYPE type; 
} rmd_motor;
#endif

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

int rmd_motor_init(rmd_motor* motor, MOTOR_TYPE type, int id, float gear_ratio, int resolution);
int rmd_motor_decoding(rmd_motor *motor, uint8_t *data);
int rmd_motor_set_brake(rmd_motor *motor, int cmd);
int rmd_motor_set_torque(rmd_motor *motor, float tor);
int rmd_motor_set_velocity(rmd_motor *motor, float vel);
int rmd_motor_set_position(rmd_motor *motor, float pos);
int rmd_motor_lock_position(rmd_motor *motor, float pos);
int rmd_motor_get_position(rmd_motor *motor);
int rmd_motor_shutdown(rmd_motor *motor);
int rmd_motor_pause(rmd_motor *motor);
int rmd_motor_resume(rmd_motor *motor);

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif // END of _rmd_motor_H_ 
