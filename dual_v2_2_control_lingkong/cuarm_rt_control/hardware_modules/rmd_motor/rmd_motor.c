/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2023 CURI & HKCLR                                   *
 * File name   : rmd_motor.c                                                *
 * Author      : CHEN Wei, Simon Tam, Black                                 *
 * Version     : 2.0.0                                                      *
 * Date        : 2023-01-01                                                 *
 * Description : Motor data structure.                                      *
 * Others      : None                                                       *
 * History     : 2022-04-05 1st version.                                    *
 *             : 2023-01-01 2nd version. Add motor type and redefine the    *
 *             :            data type.                                      *
 *             : 2026-01-01 2rd version. Divide different products.         *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#include "rmd_motor.h"
#include <stdint.h>
#include <stdio.h>

static int rmd_motor_decode_read_angle(rmd_motor *motor, const uint8_t *data)
{
    int64_t angle_raw = (int64_t)((uint64_t)data[1] |
                                  ((uint64_t)data[2] << 8) |
                                  ((uint64_t)data[3] << 16) |
                                  ((uint64_t)data[4] << 24) |
                                  ((uint64_t)data[5] << 32) |
                                  ((uint64_t)data[6] << 40) |
                                  ((uint64_t)data[7] << 48));
    if (data[7] & 0x80U) {
        angle_raw |= (int64_t)0xFF00000000000000LL;
    }
    /* 0x92 返回完整多圈绝对角度（电机侧 deg），需要除以 gear_ratio 得到输出端 deg。 */
    motor->position = (float)angle_raw * 0.01f;
    if (RMD_V4 == motor->type && motor->gear_ratio > 0.0f) {
        motor->position = motor->position / motor->gear_ratio;
    }
    motor->raw_position = motor->position;
    motor->last_position = motor->position;
    motor->has_first_data = 1;
    return 0;
}

/*
 * function: rmd_motor_init
 *     intialize the motor parameters into the motor data structure
 * input:
 *     motor[rmd_motor*]: write the motor data
 *     type[MOTOR_TYPE]: the motor type for decoding
 *     id[int]: the motor id
 *     gear_ratio[int]: the gear ratio
 *     resolution[int]: the encoder resolution
 * output:
 *     void
 */
int rmd_motor_init(rmd_motor* motor, MOTOR_TYPE type, int id, float gear_ratio, int resolution)
{
    motor->motor_id = id;
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
 *     motor[rmd_motor *]: the motor structure pointer
 * output:
 *     state[int]: return when success
 */
int rmd_motor_decoding(rmd_motor *motor, uint8_t *data)
{
    motor->motor_id = (int)(motor->cmd_id - 0x140U);

    if (data[0] == 0x92) {
        return rmd_motor_decode_read_angle(motor, data);
    }

    motor->torque = (float)((int16_t)((data[3] << 8) | data[2]));
    motor->velocity = (float)((int16_t)((data[5] << 8) | data[4]));
    const float single_turn_position = (float)((uint16_t)((data[7] << 8) | data[6]));

    if (motor->resolution <= 0 || motor->gear_ratio <= 0) {
        printf("ERROR: This motor is not initialzied \n");
        return -1;
    }

    if (RMD_V1 == motor->type) {
        motor->torque   = motor->torque   *  33.0 / 2048 / 1.6;
        motor->velocity = motor->velocity / motor->gear_ratio;
    } else if (RMD_V3 == motor->type) {
        motor->torque   = motor->torque * 0.01f;
    } else if (RMD_V4 == motor->type) {
        motor->torque   = motor->torque * 0.01f; 
        // motor->torque   = motor->torque * 66.0f / 4096.0f; // 看文档第7页
        motor->velocity = motor->velocity / motor->gear_ratio;
    }

    /* A1/A2/A4/0x8C 等应答帧末尾是单圈编码器值，不是多圈绝对位置。
     * 多圈位置只允许由 0x92 更新，避免运动命令应答污染 position/last_position。 */
    motor->raw_position = single_turn_position;
    return 0;
}

/*
 * function: rmd_motor_set_brake
 *     Run the RMD motor set brake
 */
 int rmd_motor_set_brake(rmd_motor *motor, int cmd)
 {
     motor->cmd_id = motor->motor_id + 0x140;
     motor->tx_buffer[0] = 0x8C;  // Torque command
     motor->tx_buffer[1] = cmd;
     motor->tx_buffer[2] = 0x00;
     motor->tx_buffer[3] = 0x00;
     motor->tx_buffer[4] = 0x00;
     motor->tx_buffer[5] = 0x00;
    motor->tx_buffer[6] = 0x00;
    motor->tx_buffer[7] = 0x00;
    return 0;
 }

/*
 * function: rmd_motor_set_torque
 *     Run the RMD motor in torque mode
 */
int rmd_motor_set_torque(rmd_motor *motor, float tor)
{
    // Convert torque to motor units (assuming 0.01 Nm/bit)
    int16_t value = (int16_t)(tor*100.0f);  // Scale factor
    //int16_t value = (int16_t)(tor * 4096.f / 66.0f);  // Scale factor
    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0xA1;  // Torque command
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = 0x00;
    motor->tx_buffer[3] = 0x00;
    motor->tx_buffer[4] = (value & 0xFF);
    motor->tx_buffer[5] = ((value >> 8) & 0xFF);
    motor->tx_buffer[6] = 0x00;
    motor->tx_buffer[7] = 0x00;
    return 0;
}

/*
 * function: rmd_motor_set_velocity
 *     Run the RMD motor in velocity mode
 */
int rmd_motor_set_velocity(rmd_motor *motor, float vel)
{
    // Convert velocity to motor units (assuming 0.01 dps/bit)
    if (RMD_V4 == motor->type) {
        vel = vel * motor->gear_ratio;
    }
    int32_t value = (int32_t)(vel * 100.0f);  // Scale factor
    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0xA2;  // Speed command
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = 0x00;
    motor->tx_buffer[3] = 0x00;
    motor->tx_buffer[4] = (value >> 0) & 0xFF;
    motor->tx_buffer[5] = (value >> 8) & 0xFF;
    motor->tx_buffer[6] = (value >> 16) & 0xFF;
    motor->tx_buffer[7] = (value >> 24) & 0xFF;
    return 0;
}

/*
 * function: rmd_motor_set_position
 *     Run the RMD motor in position mode
 */
int rmd_motor_set_position(rmd_motor *motor, float pos)
{
    if (RMD_V4 == motor->type) {
        pos = pos * motor->gear_ratio;
    }
    /* 0xA4: angle int32 LE (0.01°/LSB, motor-side); maxSpeed uint16 LE @ [2:3] (1 dps/LSB). */
    int32_t angle_control = (int32_t)(pos * 100.0f);
    uint16_t max_speed_dps = 1800;

    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0xA4;
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = (uint8_t)(max_speed_dps & 0xFF);
    motor->tx_buffer[3] = (uint8_t)((max_speed_dps >> 8) & 0xFF);
    motor->tx_buffer[4] = (uint8_t)((angle_control >> 0) & 0xFF);
    motor->tx_buffer[5] = (uint8_t)((angle_control >> 8) & 0xFF);
    motor->tx_buffer[6] = (uint8_t)((angle_control >> 16) & 0xFF);
    motor->tx_buffer[7] = (uint8_t)((angle_control >> 24) & 0xFF);
    return 0;
}

/*
 * function: rmd_motor_lock_position
 *     Lock motor at specified position via 0xA4 with maxSpeed=0.
 *     The drive's internal position loop holds the motor at this angle
 *     even after CAN communication stops (no host heartbeat dependency).
 */
int rmd_motor_lock_position(rmd_motor *motor, float pos)
{
    if (RMD_V4 == motor->type) {
        pos = pos * motor->gear_ratio;
    }
    /* 0xA4: angle int32 LE (0.01°/LSB, motor-side); maxSpeed uint16 LE @ [2:3] (1 dps/LSB). */
    int32_t angle_control = (int32_t)(pos * 100.0f);
    uint16_t max_speed_dps = 0;  /* 0 = no movement allowed, hold in place */

    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0xA4;
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = (uint8_t)(max_speed_dps & 0xFF);
    motor->tx_buffer[3] = (uint8_t)((max_speed_dps >> 8) & 0xFF);
    motor->tx_buffer[4] = (uint8_t)((angle_control >> 0) & 0xFF);
    motor->tx_buffer[5] = (uint8_t)((angle_control >> 8) & 0xFF);
    motor->tx_buffer[6] = (uint8_t)((angle_control >> 16) & 0xFF);
    motor->tx_buffer[7] = (uint8_t)((angle_control >> 24) & 0xFF);
    return 0;
}

int rmd_motor_get_position(rmd_motor *motor)
{
    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0x92;
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = 0x00;
    motor->tx_buffer[3] = 0x00;
    motor->tx_buffer[4] = 0x00;
    motor->tx_buffer[5] = 0x00;
    motor->tx_buffer[6] = 0x00;
    motor->tx_buffer[7] = 0x00;
    return 0;
}


/*
 * function: rmd_motor_shutdown
 *     Shutdown the RMD motor
 */
int rmd_motor_shutdown(rmd_motor *motor)
{
    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0x80;  // shutdown command
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = 0x00;
    motor->tx_buffer[3] = 0x00;
    motor->tx_buffer[4] = 0x00;
    motor->tx_buffer[5] = 0x00;
    motor->tx_buffer[6] = 0x00;
    motor->tx_buffer[7] = 0x00;
    return 0;
}

/*
 * function: rmd_motor_pause
 *     Pause the RMD motor
 */
int rmd_motor_pause(rmd_motor *motor)
{
    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0x81;  // pause command
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = 0x00;
    motor->tx_buffer[3] = 0x00;
    motor->tx_buffer[4] = 0x00;
    motor->tx_buffer[5] = 0x00;
    motor->tx_buffer[6] = 0x00;
    motor->tx_buffer[7] = 0x00;
    return 0;
}

/*
 * function: rmd_motor_resume
 *     Resume the RMD motor
 */
int rmd_motor_resume(rmd_motor *motor)
{
    motor->cmd_id = motor->motor_id + 0x140;
    motor->tx_buffer[0] = 0x88;  // resume command
    motor->tx_buffer[1] = 0x00;
    motor->tx_buffer[2] = 0x00;
    motor->tx_buffer[3] = 0x00;
    motor->tx_buffer[4] = 0x00;
    motor->tx_buffer[5] = 0x00;
    motor->tx_buffer[6] = 0x00;
    motor->tx_buffer[7] = 0x00;
    return 0;
}