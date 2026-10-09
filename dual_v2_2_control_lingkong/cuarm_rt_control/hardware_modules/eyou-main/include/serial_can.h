#ifndef SERIAL_CAN_H
#define SERIAL_CAN_H

#include "controlcan.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
#define TEST_MODE
#ifdef TEST_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

#define CH0 0x01  // 通道0的掩码
#define CH1 0x02  // 通道1的掩码


int serial_can_open(DWORD channel);
int serial_can_send(unsigned int can_id, const BYTE* data, DWORD device_id, DWORD m_can_id);
int serial_can_receive(BYTE* data, unsigned int *receive_id, DWORD device_id, DWORD m_can_id);
int serial_can_close(DWORD device_id);
#ifdef __cplusplus
}
#endif
#endif