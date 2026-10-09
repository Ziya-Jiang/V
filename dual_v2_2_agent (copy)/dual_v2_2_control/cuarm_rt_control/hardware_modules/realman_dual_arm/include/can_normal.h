#ifndef CAN_NORMAL_H
#define CAN_NORMAL_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "eu_canable.h"
#ifdef __cplusplus
extern "C"
{
#endif
typedef enum USB_CAN_DEVICE
{
    EU_CANABLE = 0,
    CAN_ANLYSTII = 1
} USB_CAN_DEVICE;


int can_normal_init(unsigned int channel, USB_CAN_DEVICE device_type);
int can_normal_send(USB_CAN_DEVICE device_type, unsigned int can_id, const unsigned char* data, unsigned int device_id, unsigned int m_can_id);
int can_normal_receive(USB_CAN_DEVICE device_type, unsigned char* data, unsigned int *receive_id, unsigned int device_id, unsigned int m_can_id);
int can_normal_close(USB_CAN_DEVICE device_type, unsigned int device_id);
#ifdef __cplusplus
}
#endif
#endif