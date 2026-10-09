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

 #include <stdio.h>
 #include <unistd.h>
 #include <stdint.h>
 #include <string.h>
 #include "cxt_can_rmd_motor.h"
 
#ifndef RMD_EXCHANGE_TX_SETTLE_US
#define RMD_EXCHANGE_TX_SETTLE_US 200
#endif

 static void cxt_can_rmd_print_frame(const char *tag, int can_idx, int slot, const VCI_CAN_OBJ *frame)
 {
     // fprintf(stderr, "[CXT][RMD] %s can=%d slot=%d ID=0x%03X len=%u data:",
     //         tag, can_idx, slot, (unsigned)frame->ID, (unsigned)frame->DataLen);
     // for (int i = 0; i < frame->DataLen && i < 8; ++i) {
     //     fprintf(stderr, " %02X", frame->Data[i]);
     // }
     // fputc('\n', stderr);
     (void)tag; (void)can_idx; (void)slot; (void)frame;
 }
 
 int cxt_can_rmd_motor_pack(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i) {
         bus->tx_buffer[i].ID = motor[i].cmd_id;
         bus->tx_buffer[i].SendType = 0;
         bus->tx_buffer[i].RemoteFlag = 0;
         bus->tx_buffer[i].ExternFlag = 0;
         bus->tx_buffer[i].DataLen = 8;
         memcpy(bus->tx_buffer[i].Data, motor[i].tx_buffer, 8);
     }
     return 0;
 }
 
 int cxt_can_rmd_motor_unpack(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
    int used[CXT_CAN_MAX_NODE_SIZE] = {0};

    for (int i = 0; i < bus->node_num[can_idx]; ++i) {
        const uint32_t expect = (uint32_t)motor[i].cmd_id & 0x7FFU;
        const uint32_t fallback = (uint32_t)(motor[i].motor_id + 0x700) & 0x7FFU;
        const uint8_t expect_cmd = motor[i].tx_buffer[0];
        int found = 0;

        for (int j = 0; j < bus->node_num[can_idx]; ++j) {
            if (used[j])
                continue;

            const uint32_t rx_id = (uint32_t)bus->rx_buffer[j].ID & 0x7FFU;
            if (rx_id != expect && rx_id != fallback)
                continue;
            if (bus->rx_buffer[j].Data[0] != expect_cmd) {
                
                continue;
            }

            used[j] = 1;
            found = 1;
            
            rmd_motor_decoding(&motor[i], bus->rx_buffer[j].Data);
            if (rx_id == fallback) {
                
            }
            break;
        }
        if (!found) {
            
            return -1;
        }
     }
     return 0;
 }
 
 int cxt_can_rmd_motor_exchange(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
     if (!bus || !motor) {
         
         return -1;
     }
     if (can_idx < 0 || can_idx >= bus->can_num) {
         
         return -1;
     }
     cxt_can_clear(bus, can_idx);
     cxt_can_rmd_motor_pack(bus, can_idx, motor);
     for (int i = 0; i < bus->node_num[can_idx]; ++i) {
         cxt_can_rmd_print_frame("tx", can_idx, i, &bus->tx_buffer[i]);
     }
     int ret = cxt_can_send(bus, can_idx);
     if (ret < 0) {
         
         return ret;
     }
     if (RMD_EXCHANGE_TX_SETTLE_US > 0) {
         usleep(RMD_EXCHANGE_TX_SETTLE_US);
     }

     ret = cxt_can_receive(bus, can_idx);
     if (ret < 0) {
         
         return ret;
     }
     for (int i = 0; i < bus->node_num[can_idx]; ++i) {
         cxt_can_rmd_print_frame("rx", can_idx, i, &bus->rx_buffer[i]);
     }
     return cxt_can_rmd_motor_unpack(bus, can_idx, motor);
 }
 
 int cxt_can_rmd_motor_enable(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
     int ret;
     for (int i = 0; i < bus->node_num[can_idx]; ++i)
         rmd_motor_resume(&motor[i]);
     ret = cxt_can_rmd_motor_exchange(bus, can_idx, motor);
     if (ret < 0)
         return ret;
     for (int i = 0; i < bus->node_num[can_idx]; ++i)
         rmd_motor_set_brake(&motor[i], 1);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }   
 
 int cxt_can_rmd_motor_disable(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i) 
         rmd_motor_set_brake(&motor[i], 0);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }
 
 int cxt_can_rmd_motor_set_torque(cxt_can *bus, int can_idx, rmd_motor *motor, float tor[])    
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i) 
         rmd_motor_set_torque(&motor[i], tor[i]);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }
 
 int cxt_can_rmd_motor_set_velocity(cxt_can *bus, int can_idx, rmd_motor *motor, float vel[])  
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i) 
         rmd_motor_set_velocity(&motor[i], vel[i]);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }
 
 int cxt_can_rmd_motor_set_position(cxt_can *bus, int can_idx, rmd_motor *motor, float pos[])  
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i) 
         rmd_motor_set_position(&motor[i], pos[i]);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }
 
 int cxt_can_rmd_motor_shutdown(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i)
         rmd_motor_shutdown(&motor[i]);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }

 int cxt_can_rmd_motor_lock_position(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i)
         rmd_motor_lock_position(&motor[i], motor[i].position);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }

 int cxt_can_rmd_motor_pause(cxt_can *bus, int can_idx, rmd_motor *motor)      
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i) 
         rmd_motor_pause(&motor[i]);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }
 
 int cxt_can_rmd_motor_resume(cxt_can *bus, int can_idx, rmd_motor *motor)
 {
     for (int i = 0; i < bus->node_num[can_idx]; ++i) 
         rmd_motor_resume(&motor[i]);
     return cxt_can_rmd_motor_exchange(bus, can_idx, motor);
 }
 