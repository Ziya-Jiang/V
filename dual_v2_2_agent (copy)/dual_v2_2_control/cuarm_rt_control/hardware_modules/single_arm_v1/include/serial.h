/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2023 CURI & HKCLR                                   *
 * File name   : serial.h                                                   *
 * Author      : CHEN Wei, Simon Tam                                        *
 * Version     : 2.0.0                                                      *
 * Date        : 2023-01-01                                                 *
 * Description : Serial port communication: open, close, read and write.    *
 * Others      : None                                                       *
 * History     : 2022-04-05 1st version.                                    *
 *             : 2023-01-01 2nd version. Change the type define and the     *
 *             :            repository name.                                *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#ifndef SERIAL_H
#define SERIAL_H

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

#ifndef SERIAL_DEFINE
#define SERIAL_DEFINE
typedef struct _serial {
#ifdef WIN32
    HANDLE hCom;
#else
    int hCom;
#endif
    int lock; // Wait until the transfer is complete before making the next transfer
}serial;
#endif

// If this is a C++ compiler, use C linkage
#ifdef __cplusplus
extern "C" {
#endif

/*
 * function: serial_open
 *     Open the serial port
 * input: 
 *     serial_port[serial *]: the serial struct pointor
 *     com_id[const char array]: the com id of the serial port
 *     baud_rate[int]: the baud rate of the serial port communication
 * output:
 *     state[int]: success return 0
 */
int serial_open(serial* serial_port, const char com_id[], int baud_rate, int byte_size, int parity, int stop_bits);

/*
 * function: serial_write
 *     Write data to the serial port
 * input: 
 *     serial_port[serial *]: the serial struct pointor
 *     write_bytes[DWORD]: the data size write to the serial port
 *     write_buffer[BYTE *]: the data buffer write to the serial port
 * output:
 *     state[int]: success return 0
 */
int serial_write(serial* serial_port, DWORD write_size, BYTE write_buffer[]);

/*
 * function: serial_read
 *     Read data from the serial port
 * input: 
 *     serial_port[serial *]: the serial struct pointor
 *     read_bytes[DWORD]: the data size read from the serial port
 *     read_buffer[BYTE *]: the data buffer read from the serial port
 * output:
 *     state[int]: success return true
 */
int serial_read(serial* serial_port, DWORD read_size, BYTE read_buffer[]);

/*
 * function: serial_crc16
 *      Do crc 16
 * input:
 *      data_packet[BYTE *]: the data to be checked
 *      length_of_data[int]: length of data to be checked sum
 *      CRC_L[BYTE *]: return the crc 16 low byte
 *      CRC_L[BYTE *]: return the crc 16 high byte
 * output:
 */
void serial_crc16(BYTE data_packet[], int len, BYTE* CRC_L, BYTE* CRC_H);

/*
 * function: serial_close
 *     close the serial port
 * input:
 *     serial_port[serial *]: the serial struct pointor
 * output: 
 *     state[int]: success return 0
 */
int serial_close(serial* serial_port);

// If this is a C++ compiler, use C linkage
#ifdef __cplusplus
}
#endif

#endif // CURI_SERIAL_H
