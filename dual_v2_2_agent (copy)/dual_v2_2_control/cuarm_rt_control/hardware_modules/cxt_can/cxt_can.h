/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2026 CURI & HKCLR                                   *
 * File name   : curi_cxt_can.h                                             *
 * Author      : CHEN Wei, Simon Tam                                        *
 * Version     : 1.0.0                                                      *
 * Date        : 2022-04-05                                                 *
 * Description : Serial port communication implementation for encoder,      *
 *             : force sensor, and motor control.                           *
 * Others      : None                                                       *
 * History     : 2022-04-05 Initial release (v1.0.0).                       *
 *             : 2026-04-25 Update name (v1.1.0).                           *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/


 #ifndef CXT_CAN_H
 #define CXT_CAN_H
 
 #include <stdint.h>
 #include "controlcan.h" 
 
 #define CXT_CAN_DEFAULT_TIMEOUT_MS 1000
 #define CXT_CAN_MAX_CHANNEL 4
 #define CXT_CAN_MAX_NODE_SIZE 16
 #define CXT_CAN_VCI_RX_BATCH 256
 
 #ifndef CXT_CAN_DEFINE
 #define CXT_CAN_DEFINE
 typedef struct _cxt_can {
     DWORD device_type;  // VCI_USBCAN2 or others
     DWORD device_id;    // Device index
     int can_num;
     int can_idx[CXT_CAN_MAX_CHANNEL];    // CAN channel (0 or 1)
     int node_num[CXT_CAN_MAX_CHANNEL]; 
     VCI_CAN_OBJ rx_buffer[CXT_CAN_MAX_NODE_SIZE];
     VCI_CAN_OBJ tx_buffer[CXT_CAN_MAX_NODE_SIZE];
 } cxt_can;
 #endif
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 int cxt_can_init(cxt_can *bus, UINT deviceType, int dev_idx, int can_num, int can_idx[], int node_num[]); 
 int cxt_can_send(cxt_can *bus, int can_idx);
 int cxt_can_receive(cxt_can *bus, int can_idx);
 void cxt_can_clear(cxt_can *bus, int can_idx);
 int cxt_can_close(cxt_can *bus);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // CXT_CAN_H