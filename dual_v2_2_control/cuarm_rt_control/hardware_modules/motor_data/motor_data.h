/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2023 CURI & HKCLR                                   *
 * File name   : motor_data.h                                               *
 * Author      : CHEN Wei, Simon Tam, Black                                 *
 * Version     : 2.0.0                                                      *
 * Date        : 2023-01-01                                                 *
 * Description : Motor data structure.                                      *
 * Others      : None                                                       *
 * History     : 2022-04-05 1st version.                                    *
 *             : 2023-01-01 2nd version. Add motor type and redefine the    *
 *             :            data type.                                      *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

// data type
#ifndef BYTE_DEFINE
#define BYTE_DEFINE
    typedef unsigned char       BYTE;
#endif

#ifndef CHAR_DEFINE
#define CHAR_DEFINE
    typedef char				CHAR;
#endif

#ifndef UCHAR_DEFINE
#define UCHAR_DEFINE
    typedef unsigned char		UCHAR;
#endif

#ifndef int16_t_DEFINE
#define int16_t_DEFINE
    typedef short int           int16_t;
#endif

#ifndef INT_DEFINE
#define INT_DEFINE
    typedef int                 INT;
#endif

#ifndef UINT_DEFINE
#define UINT_DEFINE
    typedef unsigned int        UINT;
#endif

#ifndef USHORT_DEFINE
#define USHORT_DEFINE
    typedef unsigned short		USHORT;
#endif

#ifndef ULONG_DEFINE
#define ULONG_DEFINE
    typedef unsigned long		ULONG;
#endif

#ifndef DWORD_DEFINE
#define DWORD_DEFINE
    typedef unsigned long       DWORD;
#endif

#ifndef PVOID_DEFINE
#define PVOID_DEFINE
    typedef void*               PVOID;
#endif

#ifndef MOTOR_TYPE_DEFINE
#define MOTOR_TYPE_DEFINE
typedef enum {
    RMD_V1 = 1,
    RMD_V3 = 2,
    RMD_V4 = 3,
    RMD_V4_1 = 4,
    RMD_V999 = 999,
}MOTOR_TYPE;
#endif

#ifndef MOTOR_DATA_DEFINE
#define MOTOR_DATA_DEFINE
typedef struct {
    int id, resolution;
    float gear_ratio;
    float position, velocity, torque, current;
    float raw_position;
    float last_position;
    int circle, has_first_data;
    MOTOR_TYPE type; 
} motor_data;
#endif

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

/*
 * function: motor_init
 *     Intialize the motor parameters
 * input: 
 *     motor[motor_data*]: write the motor data
 *     type[MOTOR_TYPE]: the motor type for decoding
 *     id[int]: the motor id
 *     gear_ratio[int]: the gear ratio
 *     resolution[int]: the encoder resolution
 * output:
 *     void
 */
void motor_init(motor_data* motor, MOTOR_TYPE type, int id, float gear_ratio, int decoder_resolution);

/*
 * function: motor_decoding
 *     decode motor data and deal with multi-circle issue
 * input: 
 *     motor[motor_data *]: the motor structure pointer
 * output:
 *     state[int]: return when success
 */
int motor_decoding(motor_data *motor);

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif // END of _MOTOR_DATA_H_ 
