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

#include "cxt_can_tl_motor.h"
#include "controlcan.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>
 
#define motor_size 1

int tl_motor_enable_state_machine(cxt_can *bus, int can_idx, tl_motor *motor)         
{
    motor_error_t ret = cxt_can_tl_motor_set_nmt(bus, can_idx, motor);
    if (ret != MOTOR_OK) {
        printf("[CXT] cxt_can_tl_motor_set_nmt: send NMT failed\n");
        return -1;
    }
    printf("[CXT] cxt_can_tl_motor_set_nmt: send NMT success\n");

    ret = cxt_can_tl_motor_get_status_word(bus, can_idx, motor);
    if (ret != MOTOR_OK) {
        printf("[CXT] cxt_can_tl_motor_get_status_word: get status word failed\n");
        return -1;
    }
    printf("[CXT] cxt_can_tl_motor_get_status_word: status word: %s [0x%04X]\n", tl_motor_state_to_string(motor->status_word), motor->status_word);
    
    ret = cxt_can_tl_motor_reset(bus, can_idx, motor);
    if (ret != MOTOR_OK) {
        printf("[CXT] cxt_can_tl_motor_reset: reset failed\n");
        return -1;
    }
    printf("[CXT] cxt_can_tl_motor_reset: reset success\n");

    int max_step = 10;    
    for (int i = 0; i < max_step; ++i) {
        ret = cxt_can_tl_motor_shutdown(bus, can_idx, motor);
        if (ret != MOTOR_OK) {
            printf("[CXT] cxt_can_tl_motor_shutdown: shutdown failed\n");
            return -1;
        }
        cxt_can_tl_motor_get_status_word(bus, can_idx, motor);
        printf("[CXT] cxt_can_tl_motor_enable: status word: %s [0x%04X]\n", tl_motor_state_to_string(motor->status_word), motor->status_word);
        if (motor->status_word & STATUS_READY_TO_SWITCH_ON) {
            break;
        }
        usleep(100000);
    }
    printf("[CXT] cxt_can_tl_motor_switch_on: shutdown success\n");

    for (int i = 0; i < max_step; ++i) {
        ret = cxt_can_tl_motor_switch_on(bus, can_idx, motor);
        if (ret != MOTOR_OK) {
            printf("[CXT] cxt_can_tl_motor_switch_on: switch_on failed\n");
            return -1;
        }
        cxt_can_tl_motor_get_status_word(bus, can_idx, motor);
        printf("[CXT] cxt_can_tl_motor_enable: status word: %s [0x%04X]\n", tl_motor_state_to_string(motor->status_word), motor->status_word);
        if (motor->status_word & STATUS_SWITCHED_ON) {
            break;
        }
        usleep(100000);
    }
    printf("[CXT] cxt_can_tl_motor_switch_on: switch_on success\n");

    for (int i = 0; i < max_step; ++i) {
        ret = cxt_can_tl_motor_enable(bus, can_idx, motor);
        if (ret != MOTOR_OK) {
            printf("[CXT] cxt_can_tl_motor_enable: enable failed\n");
            return -1;
        }
        cxt_can_tl_motor_get_status_word(bus, can_idx, motor);
        printf("[CXT] cxt_can_tl_motor_enable: status word: %s [0x%04X]\n", tl_motor_state_to_string(motor->status_word), motor->status_word);
        if (motor->status_word & STATUS_OPERATION_ENABLED) {
            return 0;
        }
        usleep(100000);
    }
    printf("[CXT] cxt_can_tl_motor_enable: enable success\n");
    return 0;
}

int main(void)
{
    cxt_can bus;
    int can_idx[1] = {0};
    int node_num[1] = {1};
    cxt_can_init(&bus, VCI_USBCAN2, 0, 1, can_idx, node_num);
    // tl_motor_enable_state_machine(&bus, 0, motor);
    // // 后面所有 cxt_can_tl_motor_* 的 can_idx 参数也改成 0
    
    tl_motor motor[motor_size];
    printf("[CXT] motor address: %p\n", (void *)motor);
    for (int i = 0; i < motor_size; ++i)
        tl_motor_init(motor, (uint8_t)(i + 1), MODE_POSITION, 50.0f, 100, 0.001f);

    int ret = tl_motor_enable_state_machine(&bus, can_idx[0], motor);
    if (ret != 0) {
        printf("[CXT] tl_motor_enable_state_machine: enable state machine failed\n");
        return -1;
    }
    
    FILE *fp = fopen("data.txt", "w");
    int data_size = 500;
    int delay_us = 2 * 1000;
    float pos[motor_size] = {0.0f};
    float vel[motor_size] = {0.0f};
    float tor[motor_size] = {0.0f};
    for (int i = 0; i < data_size; ++i) {       
        for (int j = 0; j < motor_size; ++j) 
            pos[j] = 45.0f * (1 - cos(2.0 * 3.1415926 * i / data_size));
        
        // 统计该段的总时间，打印出来时间、平均耗时和Hz
        #include <time.h>
        static struct timespec start_ts, end_ts;
        static double total_time = 0.0;

        clock_gettime(CLOCK_MONOTONIC, &start_ts);

        cxt_can_tl_motor_set_position(&bus, can_idx[0], motor, pos);
        cxt_can_tl_motor_get_position(&bus, can_idx[0], motor);
        cxt_can_tl_motor_get_velocity(&bus, can_idx[0], motor);
        cxt_can_tl_motor_get_torque(&bus, can_idx[0], motor);

        clock_gettime(CLOCK_MONOTONIC, &end_ts);
        double elapsed = (end_ts.tv_sec - start_ts.tv_sec) + 
                         (end_ts.tv_nsec - start_ts.tv_nsec) / 1e9;
        total_time += elapsed;

        printf("[PERF] iteration: %d, elapsed: %.6f s, avg: %.6f s, Hz: %.2f\n",
               i, elapsed, total_time/(i+1), 1.0/(total_time/(i+1)) );

        
        fprintf(fp, "%d %f %f %f %f %f %f\n", i, pos[0], vel[0], tor[0], motor[0].position, motor[0].velocity, motor[0].torque);
        // usleep(delay_us);
    }    
    // use velocity control to set position to 90 * sin(2.0 * 3.1415926 * i / data_size)
    for (int i = 0; i < data_size; ++i) {
        for (int j = 0; j < motor_size; ++j)
            vel[j] = 90.0f * sin(2.0 * 3.1415926 * i / data_size);
        cxt_can_tl_motor_set_velocity(&bus, can_idx[0], motor, vel);
        cxt_can_tl_motor_get_position(&bus, can_idx[0], motor);
        cxt_can_tl_motor_get_velocity(&bus, can_idx[0], motor);
        cxt_can_tl_motor_get_torque(&bus, can_idx[0], motor);
        fprintf(fp, "%d %f %f %f %f %f %f\n", i + data_size, pos[0], vel[0], tor[0], motor[0].position, motor[0].velocity, motor[0].torque);
        usleep(delay_us);
    }
 
    cxt_can_tl_motor_set_torque(&bus, can_idx[0], motor, tor);
    cxt_can_tl_motor_disable(&bus, can_idx[0], motor);
    cxt_can_close(&bus);
 
    fclose(fp);
    return 0;
 }
 
