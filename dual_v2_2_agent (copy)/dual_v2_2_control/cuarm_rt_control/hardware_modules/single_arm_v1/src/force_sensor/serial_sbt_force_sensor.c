/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : serial_sbt_force_sensor.c                                  *
 * Author      : CHEN Wei, Simon Tam                                        *
 * Version     : 1.0.0                                                      *
 * Date        : 2022-05-04                                                 *
 * Description : Serial port sbt module communication to read force.        *
 * Others      : None                                                       *
 * History     : 2024-01-21 1st version.                                    *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#include "serial_sbt_force_sensor.h"
#include <stdint.h>
/*
 * function: serial_sbt_force_sensor_open
 *     Open the serial port
 * input: 
 *     sensor[serial_sbt_force_sensor *]: the serial_sbt_force_sensor struct pointor
 *     com_id[const char array]: the com id of the serial port
 *     baud_rate[int]: the baud rate of the serial port communication
 *     id[int]: the motor id
 * output:
 *     state[int]: success return 0
 */
int serial_sbt_force_sensor_open(serial_sbt_force_sensor* sensor, const char com_id[], int baud_rate)
{    
    // 8 bytes for data, no parity, 2 stop_bits
    return serial_open(&(sensor->serial_port), com_id, baud_rate, 8, 0, 1);
}

/*
 * function: serial_sbt_force_sensor_read
 *     Read data from the serial port
 * input: 
 *     sensor[serial_sbt_force_sensor *]: the serial_sbt_force_sensor struct pointor
 * output:
 *     state[int]: success return 0
 */
int serial_sbt_force_sensor_read(serial_sbt_force_sensor* sensor)
{
    BYTE rx_buffer[31];
    BYTE tx_buffer[8] = {
        0x01,   //ID
        0x03,   //Command
        0x00,   //ID
        0x00,   //read multiple sensors
        0x00,   //LSB of length
        0x0D,   //HSB of length
        0x84,   //LSB of CRC16
        0x0F    //HSB of CRC16
    };
    
    //serial_crc16(tx_buffer, 6, &tx_buffer[6], &tx_buffer[7]);
    if (serial_write(&(sensor->serial_port), 8, tx_buffer)) 
        return -1;

    usleep(1000);

    if (serial_read(&(sensor->serial_port), 31, rx_buffer))
        return -2;
    // printf("rx_buffer: %02X %02X %02X %02X\n", rx_buffer[3], rx_buffer[4], rx_buffer[5], rx_buffer[6]);
    uint32_t temp = (uint32_t)(rx_buffer[3] << 24 | rx_buffer[4] << 16 | rx_buffer[5] << 8 | rx_buffer[6] << 0);
    memcpy(&(sensor->data), &temp, sizeof(float));
    
    return 0;
}


/*
 * function: serial_sbt_force_sensor_close
 *     close the serial port
 * input:
 *     sensor[serial_sbt_force_sensor *]: the serial_sbt_force_sensor struct pointor
 * output: 
 *     state[int]: success return 0
 */
int serial_sbt_force_sensor_close(serial_sbt_force_sensor* sensor) 
{
    return serial_close(&(sensor->serial_port));
}
