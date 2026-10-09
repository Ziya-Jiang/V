/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : linux_main.c                                               *
 * Author      : CHEN Wei, Simon Tam, Black                                 *
 * Version     : 4.0.0                                                      *
 * Date        : 2026-05-06                                                 *
 * Description : Test program for RMD motor control (classic CAN)            *
 * Others      : None                                                       *
 * History     : 2026-05-06 Use cxt_can_init + rmd_motor (match CAN-FD demo). *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#include "rmd_motor.h"
#include "cxt_can_rmd_motor.h"
#include "cxt_can.h"
#include "controlcan.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#define motor_size 1

int main(void)
{
    cxt_can bus;
    int can_idx[2] = {0, 1};
    int node_num[2] = {1, 1};
    if (cxt_can_init(&bus, VCI_USBCAN2, 0, 1, can_idx, node_num) < 0) {
        printf("[CXT] cxt_can_init: CAN init failed\n");
        return -1;
    }   

    rmd_motor motor[motor_size];
    int cmd[motor_size] = {1};
    for (int i = 0; i < motor_size; ++i) {
        rmd_motor_init(&motor[i], RMD_V4, i + 1, 10.0f, 16384);
    }
    cxt_can_rmd_motor_enable(&bus, can_idx[0], motor);
    
    FILE *fp = fopen("data.txt", "w");
    int data_size = 500;
    float pos[motor_size] = {0.0f};
    float vel[motor_size] = {0.0f};
    float tor[motor_size] = {0.0f};
    int delay_us = 10 * 1000;
    for (int i = 0; i < data_size; ++i) {       
        for (int j = 0; j < motor_size; ++j) { 
            pos[j] = 45.0f * (1 - cos(2.0 * 3.1415926 * i / data_size));
        }
        cxt_can_rmd_motor_set_position(&bus, can_idx[0], motor, pos);
        fprintf(fp, "%d %f %f %f %f %f %f\n", i, pos[0], vel[0], tor[0], motor[0].position, motor[0].velocity, motor[0].torque);
        usleep(delay_us);
    }    
    // use velocity control to set position to 90 * sin(2.0 * 3.1415926 * i / data_size)
    for (int i = 0; i < data_size; ++i) {
        for (int j = 0; j < motor_size; ++j) { 
            vel[j] = 90.0f * sin(2.0 * 3.1415926 * i / data_size);
        }
        cxt_can_rmd_motor_set_velocity(&bus, can_idx[0], motor, vel);
        fprintf(fp, "%d %f %f %f %f %f %f\n", i + data_size, pos[0], vel[0], tor[0], motor[0].position, motor[0].velocity, motor[0].torque);
        usleep(delay_us);
    }

    cxt_can_rmd_motor_set_torque(&bus, can_idx[0], motor, tor);
    fprintf(fp, "%d %f %f %f %f %f %f\n", data_size * 2, pos[0], vel[0], tor[0], motor[0].position, motor[0].velocity, motor[0].torque);

    cxt_can_rmd_motor_disable(&bus, can_idx[0], motor);
    cxt_can_close(&bus);

    fclose(fp);
    return 0;
}
