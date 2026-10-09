/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : serial_sbt_force_sensor.h                                  *
 * Author      : CHEN Wei, Simon Tam                                        *
 * Version     : 1.0.0                                                      *
 * Date        : 2024-01-21                                                 *
 * Description : Serial port sbt module communication to read force.        *
 * Others      : None                                                       *
 * History     : 2024-01-21 1st version.                                    *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#ifndef SERIAL_SBT_FORCE_SENSOR_H
#define SERIAL_SBT_FORCE_SENSOR_H

#include "serial.h"

typedef struct _serial_sbt_force_sensor {
    serial serial_port;
    float  data;
}serial_sbt_force_sensor;

#ifdef __cplusplus
extern "C" {
#endif
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
int serial_sbt_force_sensor_open(serial_sbt_force_sensor* sensor, const char com_id[], int baud_rate);
/*
 * function: serial_sbt_force_sensor_read
 *     Read data from the serial port
 * input: 
 *     sensor[serial_sbt_force_sensor *]: the serial_sbt_force_sensor struct pointor
 * output:
 *     state[int]: success return 0
 */
int serial_sbt_force_sensor_read(serial_sbt_force_sensor* sensor);

/*
 * function: serial_sbt_force_sensor_close
 *     close the serial port
 * input:
 *     sensor[serial_sbt_force_sensor *]: the serial_sbt_force_sensor struct pointor
 * output: 
 *     state[int]: success return 0
 */
int serial_sbt_force_sensor_close(serial_sbt_force_sensor* sensor);
#ifdef __cplusplus
}
#endif

#endif
