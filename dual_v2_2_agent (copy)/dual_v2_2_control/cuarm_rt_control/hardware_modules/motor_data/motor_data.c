/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2023 CURI & HKCLR                                   *
 * File name   : motor_data.c                                               *
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

#include <stdio.h>

#include "motor_data.h"

/*
 * function: motor_init
 *     intialize the motor parameters into the motor data structure
 * input: 
 *     motor[motor_data*]: write the motor data
 *     type[MOTOR_TYPE]: the motor type for decoding
 *     id[int]: the motor id
 *     gear_ratio[int]: the gear ratio
 *     resolution[int]: the encoder resolution
 * output:
 *     void
 */
void motor_init(motor_data* motor, MOTOR_TYPE type, int id, float gear_ratio, int resolution)
{
    motor->id = id;     
    motor->type = type;     
    motor->gear_ratio = gear_ratio;
    motor->resolution = resolution;
    motor->circle = 0;
    motor->has_first_data = 0;
}

/*
 * function: motor_decoding
 *     decode motor data and deal with multi-circle issue
 * input: 
 *     motor[motor_data *]: the motor structure pointer
 * output:
 *     state[int]: return when success
 */
int motor_decoding(motor_data *motor) 
{
    if (motor->resolution <= 0 || motor->gear_ratio <= 0) {
        printf("ERROR: This motor is not initialzied \n");
        return -1;
    }

    if (RMD_V1 == motor->type) {
        motor->torque   = motor->torque   *  33.0 / 2048 / 1.6;
        motor->velocity = motor->velocity / motor->gear_ratio;
        motor->position = motor->position * 360.0 / motor->gear_ratio / motor->resolution;
    } else if (RMD_V3 == motor->type) {
        motor->torque   = motor->torque * 0.01;
    }
   
    motor->raw_position = motor->position;
    if (0 == motor->has_first_data) {
        motor->last_position = motor->position;
        motor->circle = 0;
        motor->has_first_data = 1;
    } else {
        if (motor->position + motor->circle * 360.0 / motor->gear_ratio - motor->last_position < -180.0 / motor->gear_ratio)
            motor->circle ++;
        else if (motor->position + motor->circle * 360.0 / motor->gear_ratio - motor->last_position > 180.0 / motor->gear_ratio)
            motor->circle --;
        motor->position = motor->position + motor->circle * 360.0 / motor->gear_ratio;
        motor->last_position = motor->position;
    }
}