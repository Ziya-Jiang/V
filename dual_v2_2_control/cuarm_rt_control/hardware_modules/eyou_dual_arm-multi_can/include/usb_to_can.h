#ifndef USB_TO_CAN_H
#define USB_TO_CAN_H
#include <iostream>
#include "eu_harmonic.h"
#include "eu_canable.h"
#include <vector>
#include "controlcan.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#define CH0 0x01  // 通道0的掩码
#define CH1 0x02  // 通道1的掩码

void canbox_init(const std::vector<int>& channels);
void canbox_close(const std::vector<int>& channels);
int serial_can_open(DWORD channel);
int serial_can_send(unsigned int can_id, const BYTE* data, DWORD device_id, DWORD m_can_id);
int serial_can_receive(BYTE* data, unsigned int *receive_id, DWORD device_id, DWORD m_can_id);
int serial_can_close(DWORD device_id);

#endif