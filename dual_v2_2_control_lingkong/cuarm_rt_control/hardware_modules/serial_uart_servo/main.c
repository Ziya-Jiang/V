// main.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "serial.h"     
#include "serial_uart_servo.h" 

int main(void)
{
    /* ===== 初始化串口 ===== */
    char com_id[] = "/dev/ttyUSB0"; // Linux 下的串口号
    serial_uart_servo motor;

    if (serial_uart_servo_open(&motor, com_id, 115200) != 0) {
        fprintf(stderr, "Failed to open serial: %s\n", com_id);
        return -1;
    }

    serial_uart_servo_set_position(&motor, 1, 2200, 1000);
    usleep(1000000);
    serial_uart_servo_set_position(&motor, 1, 1800, 1000);

    serial_uart_servo_close(&motor);
    return 0;
}
