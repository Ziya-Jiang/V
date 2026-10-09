/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : cxt_can_rmd_motor.c                                        *
 * Author      : CHEN Wei, Simon Tam, Black                                 *
 * Version     : 4.0.0                                                      *
 * Date        : 2026-05-06                                                 *
 * Description : RMD motor helpers for Chuangxin classic CAN (ControlCAN).  *
 * Others      : None                                                       *
 * History     : 2026-05-06 Align implementation with cxt_canfd_rmd_motor.  *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#include "cxt_can_tl_motor.h"
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>

#define TL_SDO_MS 1500u

static uint32_t cxt_tl_motor_frame_id(const VCI_CAN_OBJ *frame)
{
    return (uint32_t)frame->ID & 0x7FFU;
}

static void cxt_tl_motor_decode_rx(tl_motor *motor)
{
    const uint16_t index = (uint16_t)motor->rx_buffer[1] | ((uint16_t)motor->rx_buffer[2] << 8);

    switch (index) {
    case OD_STATUS_WORD:
        motor->status_word =
            (uint16_t)(motor->rx_buffer[4] | ((uint16_t)motor->rx_buffer[5] << 8));
        break;
    case OD_ERROR_CODE:
        motor->error_code =
            (uint16_t)(motor->rx_buffer[4] | ((uint16_t)motor->rx_buffer[5] << 8));
        break;
    case OD_OP_MODE:
    case OD_OP_MODE_DISP:
        motor->mode = (operation_mode_t)motor->rx_buffer[4];
        break;
    case OD_ACTUAL_POS:
        motor->position = (float)(int32_t)(motor->rx_buffer[4] |
            ((uint32_t)motor->rx_buffer[5] << 8) | ((uint32_t)motor->rx_buffer[6] << 16) |
            ((uint32_t)motor->rx_buffer[7] << 24));
        break;
    case OD_ACTUAL_VEL:
        motor->velocity = (float)(int32_t)(motor->rx_buffer[4] |
            ((uint32_t)motor->rx_buffer[5] << 8) | ((uint32_t)motor->rx_buffer[6] << 16) |
            ((uint32_t)motor->rx_buffer[7] << 24));
        break;
    case OD_ACTUAL_TORQUE:
        motor->torque =
            (float)(int16_t)(motor->rx_buffer[4] | ((int16_t)motor->rx_buffer[5] << 8));
        break;
    default:
        break;
    }
}

uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

int cxt_can_tl_motor_pack(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) {   
        bus->tx_buffer[i].ID = 0x600 + motor[i].tx_id;
        bus->tx_buffer[i].SendType = 0;
        bus->tx_buffer[i].RemoteFlag = 0;
        bus->tx_buffer[i].ExternFlag = 0;
        bus->tx_buffer[i].DataLen = motor[i].tx_len;
        for (int j = 0; j < motor[i].tx_len; ++j) 
            bus->tx_buffer[i].Data[j] = motor[i].tx_buffer[j];
    }
    return 0;
}

int cxt_can_tl_motor_unpack(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    int used[CXT_CAN_MAX_NODE_SIZE] = {0};
    const int n_node = bus->node_num[chn_idx];

    for (int i = 0; i < n_node; ++i) {
        const uint32_t expect_rx = 0x580U + (uint32_t)motor[i].tx_id;
        int found = 0;

        for (int j = 0; j < n_node; ++j) {
            if (used[j]) {
                continue;
            }
            if (cxt_tl_motor_frame_id(&bus->rx_buffer[j]) != expect_rx) {
                continue;
            }

            used[j] = 1;
            found = 1;
            motor[i].rx_id = motor[i].tx_id;
            motor[i].rx_len = bus->rx_buffer[j].DataLen;
            for (int b = 0; b < motor[i].rx_len; ++b) {
                motor[i].rx_buffer[b] = bus->rx_buffer[j].Data[b];
            }
            cxt_tl_motor_decode_rx(&motor[i]);
            break;
        }

        if (!found) {
            fprintf(stderr, "[CXT] tl_motor_unpack: no reply for node %u (expect ID 0x%03X)\n",
                    (unsigned)motor[i].tx_id, (unsigned)expect_rx);
            return -1;
        }
    }
    return 0;
}

int cxt_can_tl_motor_exchange(cxt_can *bus, int chn_idx, tl_motor *motor)   
{
    cxt_can_tl_motor_pack(bus, chn_idx, motor);
    cxt_can_clear(bus, chn_idx);
    int ret = cxt_can_send(bus, chn_idx);
    if (ret < 0) {
        fprintf(stderr, "[CXT] Fail to set value\n");
        return ret;
    }
    ret = cxt_can_receive(bus, chn_idx);
    if (ret < 0) {
        fprintf(stderr, "[CXT] Fail to receive value\n");
        return ret;
    }
    ret = cxt_can_tl_motor_unpack(bus, chn_idx, motor);
    if (ret < 0) {
        fprintf(stderr, "[CXT] Fail to unpack TL motor reply\n");
        return ret;
    }
    return 0;
}


motor_error_t cxt_can_tl_motor_set_nmt(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    printf("[CXT] cxt_can_tl_motor_set_nmt: %d %d\n", chn_idx, bus->node_num[chn_idx]);
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) {
        bus->tx_buffer[i].ID = 0x000;
        bus->tx_buffer[i].SendType = 0;
        bus->tx_buffer[i].RemoteFlag = 0;
        bus->tx_buffer[i].ExternFlag = 0;
        bus->tx_buffer[i].DataLen = 2;
        bus->tx_buffer[i].Data[0] = 0x01;
        bus->tx_buffer[i].Data[1] = motor->tx_id;
    }
    return cxt_can_send(bus, chn_idx);
}

motor_error_t cxt_can_tl_motor_reset(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) 
        tl_motor_fault_reset(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_shutdown(cxt_can *bus, int chn_idx, tl_motor *motor)         
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) 
        tl_motor_set_shutdown(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_switch_on(cxt_can *bus, int chn_idx, tl_motor *motor)         
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) 
        tl_motor_set_switch_on(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_enable(cxt_can *bus, int chn_idx, tl_motor *motor)         
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) 
        tl_motor_set_enable(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_disable(cxt_can *bus, int chn_idx, tl_motor *motor)         
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) 
        tl_motor_set_disable(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_set_torque(cxt_can *bus, int chn_idx, tl_motor *motor, float torque[])    
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i) 
        tl_motor_set_torque(&motor[i], torque[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_set_velocity(cxt_can *bus, int chn_idx, tl_motor *motor, float velocity[])  
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_set_velocity(&motor[i], velocity[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_set_position(cxt_can *bus, int chn_idx, tl_motor *motor, float position[])  
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_set_position(&motor[i], position[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_set_max_velocity(cxt_can *bus, int chn_idx, tl_motor *motor, float max_velocity[])
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_set_max_velocity(&motor[i], max_velocity[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_set_operation_mode(cxt_can *bus, int chn_idx, tl_motor *motor,
                                                   operation_mode_t mode[])
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_set_operation_mode(&motor[i], mode[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_get_operation_mode(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_get_operation_mode(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_get_operation_mode_display(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_get_operation_mode_display(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_get_status_word(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_get_status_word(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}

motor_error_t cxt_can_tl_motor_get_position(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_get_position(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);  
}

motor_error_t cxt_can_tl_motor_get_velocity(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_get_velocity(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);  
}

motor_error_t cxt_can_tl_motor_get_torque(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_get_torque(&motor[i]); // sent the read command to the motor       
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);  // exchange the data with the motor  
}

motor_error_t cxt_can_tl_motor_get_error_code(cxt_can *bus, int chn_idx, tl_motor *motor)
{
    for (int i = 0; i < bus->node_num[chn_idx]; ++i)
        tl_motor_get_error_code(&motor[i]);
    return cxt_can_tl_motor_exchange(bus, chn_idx, motor);
}
