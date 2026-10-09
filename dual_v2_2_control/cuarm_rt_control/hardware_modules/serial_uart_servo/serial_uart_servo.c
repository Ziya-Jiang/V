#include "serial_uart_servo.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

int serial_uart_servo_open(serial_uart_servo* motor, const char com_id[], int baud_rate)
{    
    // 8 bytes for data, no parity, 1 stop_bits
    return serial_open(&(motor->serial_port), com_id, baud_rate, 8, 0, 1);
}

int serial_uart_servo_set_mode_servo(serial_uart_servo* motor)
{
    BYTE tx_buffer[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1C, 0x01, 0xDA};
    return serial_write(&(motor->serial_port), 8, tx_buffer);
}

int serial_uart_servo_set_position(serial_uart_servo* motor, BYTE id, float pos, float time)
{
    BYTE tx_buffer[11];
	tx_buffer[0] = 0xFF; // header
	tx_buffer[1] = 0xFF; // header
	tx_buffer[2] = id; // id
    tx_buffer[3] = 0x07; // length
    tx_buffer[4] = 0x03; // command: write
    tx_buffer[5] = 0x2A; // address: goal position
    int16_t pos_int = (int16_t)pos;
    tx_buffer[6] = (BYTE)((pos_int >> 8) & 0xFFFF); // position high byte
	tx_buffer[7] = (BYTE)(pos_int & 0xFF); // position low byte
    int16_t time_int = (int16_t)time;
    tx_buffer[8] = (BYTE)((time_int >> 8) & 0xFFFF); // time high byte
    tx_buffer[9] = (BYTE)(time_int & 0xFF); // time low byte
    tx_buffer[10] = 0x00; // checksum
    for (int i = 2; i < 10; ++ i)
        tx_buffer[10] += tx_buffer[i]; 
    tx_buffer[10] = (~tx_buffer[10]) & 0xFF; 
    return serial_write(&(motor->serial_port), 11, tx_buffer);
}

int serial_uart_servo_get_position(serial_uart_servo* motor, BYTE id)
{
    BYTE tx_buffer[8];
	tx_buffer[0] = 0xFF; // header
	tx_buffer[1] = 0xFF; // header
	tx_buffer[2] = id; // id
    tx_buffer[3] = 0x04; // length
    tx_buffer[4] = 0x02; // command: read
    tx_buffer[5] = 0x38; // address: goal position
    tx_buffer[6] = 0x02; // position high byte
    tx_buffer[7] = 0x00; // checksum
    for (int i = 2; i < 7; ++ i)
        tx_buffer[7] += tx_buffer[i]; 
    tx_buffer[7] = (~tx_buffer[7]) & 0xFF; 
    return serial_write(&(motor->serial_port), 8, tx_buffer);

    // BYTE rx_buffer[8];
    // for (int i =0; i < 10; i++){
    //     serial_write(&(motor->serial_port), 8, tx_buffer);
    //     usleep(2000);
    //     if (serial_read(&(motor->serial_port), 8, rx_buffer) == 0){
    //         printf("%hhx\n", rx_buffer[5]);
    //         printf("%hhx\n", rx_buffer[6]);
    //         int position = rx_buffer[5] | (rx_buffer[6] << 8);
    //         printf("position: %d\n", position);
    //     }
    // }
    // return 0;
}

int serial_uart_servo_close(serial_uart_servo* motor)
{
    return serial_close(&(motor->serial_port));
}
