/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : windows_main.c                                             *
 * Author      : CHEN Wei                                                   *
 * Version     : 4.0.0                                                      *
 * Date        : 2026-05-06                                                 *
 * Description : Test demo for RMD motor (classic CAN, ControlCAN).          *
 * Others      : None                                                       *
 * History     : 2026-05-06 Align with cxt_can_init + rmd_motor.              *
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
#include <windows.h>

int main(void)
{
    cxt_can can_bus;
    int can_idx[2] = {0, 0};
    if (cxt_can_init(&can_bus, VCI_USBCAN2, 0, 1, can_idx) != 0) {
        printf("CAN init failed\n");
        return -1;
    }

    cxt_can_motor_ctx ctx = { &can_bus, 0 };
    rmd_motor motor;
    rmd_motor_init(&ctx, &motor, RMD_V1, 1, 10.0f, 16384);

    cxt_can_rmd_motor_set_velocity(&motor, 0.0f);

    int data_size = 500;
    int delay_ms = 1;
    for (int i = 0; i < data_size; ++i) {
        cxt_can_rmd_motor_set_velocity(&motor,
            (float)(90.0 * sin(2.0 * 3.1415926 * i / data_size)));
        Sleep(delay_ms);
        if (i % 50 == 0) {
            printf("%d motor: pos = %f, vel = %f, tor = %f\n",
                   i, motor.position, motor.velocity, motor.torque);
        }
    }

    cxt_can_rmd_motor_set_velocity(&motor, 0.0f);
    cxt_can_close(can_bus.device_type, can_bus.device_id);
    return 0;
}
