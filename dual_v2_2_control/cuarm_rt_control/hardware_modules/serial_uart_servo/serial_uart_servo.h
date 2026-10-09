#ifndef UART_SERVO_H
#define UART_SERVO_H

#include "serial.h"

typedef struct _serial_uart_servo {
    serial serial_port;
    DWORD  id;
    float  pos, time;
}serial_uart_servo;

int serial_uart_servo_open(serial_uart_servo* motor, const char com_id[], int baud_rate);
int serial_uart_servo_set_mode_servo(serial_uart_servo* motor);
int serial_uart_servo_set_position(serial_uart_servo* motor, BYTE id, float pos, float time);
int serial_uart_servo_get_position(serial_uart_servo* motor, BYTE id);
int serial_uart_servo_close(serial_uart_servo* motor);

#endif