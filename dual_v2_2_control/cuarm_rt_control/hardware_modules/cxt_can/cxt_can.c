/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2026 CURI & HKCLR                                   *
 * File name   : curi_cxt_can.c                                             *
 * Author      : CHEN Wei, Simon Tam                                        *
 * Version     : 1.0.0                                                      *
 * Date        : 2022-04-05                                                 *
 * Description : Serial port communication implementation for encoder,      *
 *               force sensor, and motor control.                           *
 * Others      : None                                                       *
 * History     : 2022-04-05 Initial release (v1.0.0).                       *
 *             : 2026-04-25 Update name (v1.1.0).                           *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

 #include "cxt_can.h"
 #include "stdio.h"
 #include <stdint.h>
 #include <string.h>
 #include <unistd.h>

 static DWORD cxt_can_hw_channel(const cxt_can *bus, int chn_idx)
 {
     if (!bus || chn_idx < 0 || chn_idx >= bus->can_num) {
         return 0;
     }
     return (DWORD)bus->can_idx[chn_idx];
 }

 static uint32_t cxt_can_frame_id(const VCI_CAN_OBJ *frame)
 {
     return (uint32_t)frame->ID & 0x7FFU;
 }

 static int cxt_can_tx_is_sdo_client(uint32_t tx_id)
 {
     const uint32_t id = tx_id & 0x7FFU;
     return id >= 0x600U && id <= 0x67FU;
 }

 static uint32_t cxt_can_sdo_expected_rx_id(uint32_t tx_id)
 {
     return (tx_id & 0x7FFU) - 0x80U;
 }

 static int cxt_can_use_sdo_rx_match(const cxt_can *bus, int chn_idx)
 {
     const int n_need = bus->node_num[chn_idx];
     for (int i = 0; i < n_need; ++i) {
         if (!cxt_can_tx_is_sdo_client(bus->tx_buffer[i].ID)) {
             return 0;
         }
     }
     return n_need > 0;
 }

 static int cxt_can_receive_fifo(cxt_can *bus, int chn_idx, DWORD hw_chn)
 {
     VCI_CAN_OBJ batch[CXT_CAN_VCI_RX_BATCH];
     int n_rx = 0;
     const int n_need = bus->node_num[chn_idx];

     for (int r = 0; r < 100 && n_rx < n_need; ++r) {
         const int ret = VCI_Receive(bus->device_type, bus->device_id, hw_chn, batch, n_need, 100);
         if (ret == 0) {
             usleep(1000);
             continue;
         }
         for (int k = 0; k < ret && n_rx < n_need; ++k) {
             if (batch[k].RemoteFlag != 0 || batch[k].DataLen == 0) {
                 continue;
             }
             bus->rx_buffer[n_rx] = batch[k];
             n_rx++;
         }
     }

     if (n_rx != n_need) {
         fprintf(stderr, "[CXT] cxt_can_receive: %d/%d replies from CAN %d\n", n_rx, n_need, chn_idx);
         return -2;
     }
     return 0;
 }

 static int cxt_can_receive_sdo_match(cxt_can *bus, int chn_idx, DWORD hw_chn)
 {
     VCI_CAN_OBJ batch[CXT_CAN_VCI_RX_BATCH];
     int filled[CXT_CAN_MAX_NODE_SIZE] = {0};
     const int n_need = bus->node_num[chn_idx];
     int n_filled = 0;

     for (int r = 0; r < 100 && n_filled < n_need; ++r) {
         const int ret = VCI_Receive(bus->device_type, bus->device_id, hw_chn, batch, n_need, 100);
         if (ret == 0) {
             usleep(1000);
             continue;
         }
         for (int k = 0; k < ret; ++k) {
             if (batch[k].RemoteFlag != 0 || batch[k].DataLen == 0) {
                 continue;
             }
             const uint32_t rx_id = cxt_can_frame_id(&batch[k]);
             for (int i = 0; i < n_need; ++i) {
                 if (filled[i]) {
                     continue;
                 }
                 if (rx_id != cxt_can_sdo_expected_rx_id(bus->tx_buffer[i].ID)) {
                     continue;
                 }
                 bus->rx_buffer[i] = batch[k];
                 filled[i] = 1;
                 n_filled++;
                 break;
             }
         }
     }

     if (n_filled != n_need) {
         fprintf(stderr, "[CXT] cxt_can_receive: %d/%d matched SDO replies from CAN %d\n", n_filled, n_need,
                 chn_idx);
         return -2;
     }
     return 0;
 }
 
 int cxt_can_init(cxt_can *bus, UINT deviceType, int dev_idx, int can_num, int can_idx[], int node_num[])
 {
     bus->device_type = deviceType;
     bus->device_id = dev_idx;
     bus->can_num = can_num;
     for (int i = 0; i < can_num; ++i) {
         bus->can_idx[i] = can_idx[i];
         bus->node_num[i] = node_num[i];
     }
 
     printf("[CXT] Initial CAN (device: %u): %d channel(s)", dev_idx, can_num);
     for (int i = 0; i < can_num; ++i) {
         bus->can_idx[i] = can_idx[i];
         printf("%s %d: %d node(s)", i ? "," : ":", can_idx[i], node_num[i]);
     }
     printf("\n");
 
     DWORD dwRel = VCI_OpenDevice(deviceType, dev_idx, 0);
     if (dwRel != 1) {
         printf("Open Device Failed\n");
         return -1;
     }
 
     VCI_BOARD_INFO vbi;
     dwRel = VCI_ReadBoardInfo(deviceType, dev_idx, &vbi);
     if (dwRel != 1) {
         printf("Get Board Info Failed\n");
         return -2;
     }
     printf("Get VCI_ReadBoardInfo success!\n");
     printf("hw_Type:%c \n", vbi.str_hw_Type[0]);
 
     VCI_INIT_CONFIG vic;
     memset(&vic, 0, sizeof(vic));
     vic.AccCode = 0x00000000;
     vic.AccMask = 0xFFFFFFFF;
     vic.Reserved = 0;
     vic.Filter = 1;
     vic.Timing0 = 0x00;
     vic.Timing1 = 0x14;
     vic.Mode = 0;
 
     for (int i = 0; i < can_num; ++i) {
         dwRel = VCI_InitCAN(deviceType, dev_idx, can_idx[i], &vic);
         if (dwRel != 1) {
             VCI_CloseDevice(deviceType, dev_idx);
             printf("Failed to Init CAN \n");
             return -3;
         }
         printf("Init CAN %u .. OK\n", can_idx[i]);
         if (VCI_StartCAN(deviceType, dev_idx, can_idx[i]) != 1) {
             VCI_CloseDevice(deviceType, dev_idx);
             printf("Failed to Start CAN \n");
             return -4;
         }
         printf("Start CAN %u .. OK\n", (unsigned)can_idx[i]);
         printf("[CXT] CAN channel %d initialized (1000 kbit/s)\n", can_idx[i]);
     }
 
     return 0;
 }
 
 
 void cxt_can_clear(cxt_can *bus, int can_idx)
 {
     if (!bus || can_idx < 0 || can_idx >= bus->can_num)
         return;
     VCI_ClearBuffer(bus->device_type, bus->device_id, cxt_can_hw_channel(bus, can_idx));
     return;
 }
 
 /*
  * Function    : cxt_can_data_exchange
  * Description : Transmits commands to and receives data from RMD motor 
  *               in torque mode over CAN bus.
  * Input       :
  *     deviceType    : Device type identifier
  *     device_id     : Device ID
  *     can_id        : CAN channel ID
  *     cmds[]        : Command bytes to transmit (8 bytes expected)
  * Output      :
  *     data[]        : Received data bytes (8 bytes maximum)
  *     Returns 0 on success, negative error code on failure.
  */
 int cxt_can_send(cxt_can *bus, int can_idx)
 {
     if (!bus || can_idx < 0 || can_idx >= bus->can_num)
         return -1;
     const DWORD hw_chn = cxt_can_hw_channel(bus, can_idx);
     int ret = VCI_Transmit(bus->device_type, bus->device_id, hw_chn, bus->tx_buffer, bus->node_num[can_idx]);
     if (ret != bus->node_num[can_idx]) {
         fprintf(stderr, "[CXT] cxt_can_node_send: failed to send %d nodes\n", bus->node_num[can_idx]);   
         return -1;
     }
     return 0;
 }
 
 int cxt_can_receive(cxt_can *bus, int can_idx)
 {
     if (!bus || can_idx < 0 || can_idx >= bus->can_num)
         return -1;

     const DWORD hw_chn = cxt_can_hw_channel(bus, can_idx);
     if (cxt_can_use_sdo_rx_match(bus, can_idx)) {
         return cxt_can_receive_sdo_match(bus, can_idx, hw_chn);
     }
     return cxt_can_receive_fifo(bus, can_idx, hw_chn);
 }
 
 /*
  * Function    : cxt_can_close
  * Description : Closes the CAN serial port connection.
  * Input       :
  *     deviceType    : Device type identifier
  *     m_device_id   : Device ID
  * Output      :
  *     Returns 0 on success, non-zero on failure.
  */
 int cxt_can_close(cxt_can *bus)
 {
     if (!bus) {
         fprintf(stderr, "[CXT] cxt_can_close: bus is NULL\n");
         return -1;
     }
     for (int i = 0; i < bus->can_num; ++i) {
         VCI_ResetCAN(bus->device_type, bus->device_id, (DWORD)bus->can_idx[i]);
     }
     VCI_CloseDevice(bus->device_type, bus->device_id);
     printf("[CXT] CAN closed\n");
     return 0;
 }