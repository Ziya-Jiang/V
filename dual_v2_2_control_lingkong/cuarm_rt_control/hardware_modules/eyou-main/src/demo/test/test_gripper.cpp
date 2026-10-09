// main.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>     // For usleep on Linux
#include <iostream>


extern "C" 
{
    #include "serial.h"     
    #include "serial_uart_servo.h" 
}

int main(void)
{
    /* ===== 初始化串口 ===== */
    char com_id[] = "/dev/ttyUSB0"; // Linux 下的串口号
    serial_uart_servo motor;
    int set_pos, read_pos;

    if (serial_uart_servo_open(&motor, com_id, 115200) != 0) {
        fprintf(stderr, "Failed to open serial: %s\n", com_id);
        return -1;
    }
    
    set_pos = 3000;
    serial_uart_servo_set_position(&motor, 1, set_pos, 1000);
    usleep(1500000);
    // read_pos = serial_uart_servo_get_position(&motor, 1);
    // std::cout << "Set pos: " << set_pos <<", Read pos: " <<  read_pos << std::endl;

    set_pos = 450;
    serial_uart_servo_set_position(&motor, 1, set_pos, 1000);
    usleep(1500000);
    // // read_pos = serial_uart_servo_get_position(&motor, 1);
    // // std::cout << "Set pos: " << set_pos <<", Read pos: " <<  read_pos << std::endl;

    serial_uart_servo_close(&motor);
    return 0;
}