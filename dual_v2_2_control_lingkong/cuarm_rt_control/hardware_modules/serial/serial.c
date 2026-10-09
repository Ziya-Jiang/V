/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2023 CURI & HKCLR                                   *
 * File name   : serial.c                                                   *
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

#include "serial.h"
#include <stdio.h>
#ifdef WIN32
    #include "windows.h"
#else
    #include <fcntl.h>   // Contains file controls like O_RDWR
    #include <errno.h>   // Error integer and strerror() function
    #include <termios.h> // Contains POSIX terminal control definitions
    #include <unistd.h>  // write(), read(), close()
    #include <stdint.h>
    #include <string.h>
#endif

// configure parameters
#define SERVO_MOTOR_READ_BUFFER_SIZE  32
#define SERVO_MOTOR_WRITE_BUFFER_SIZE 32

/*
 * function: serial_open
 *     Open the serial port
 * input: 
 *     serial_port[serial *]: the serial struct pointor
 *     com_id[const char array]: the com id of the serial port
 *     baud_rate[int]: the baud rate of the serial port communication
 *     byte_size[int]: the byte_size of serial port
 *     parity[int]: the parity of serial port
 *     stop_bits[int]: the stop_bits of serial port
 * output:
 *     state[int]: success return 0
 */
int serial_open(serial* serial_port, const char com_id[], int baud_rate, int byte_size, int parity, int stop_bits)
{
#ifdef WIN32
    serial_port->hCom = CreateFile((LPCSTR)com_id,
                      GENERIC_READ | GENERIC_WRITE, // open mode: read & write  
                      0, //独占方式  
                      NULL,
                      OPEN_EXISTING, //打开而不是创建  
                      0, //同步方式  
                      NULL);
    if (serial_port->hCom == (HANDLE)-1) {
      	printf("Error: open serial port fail!");
        return -1;
    }

    // set buffer size
    SetupComm(serial_port->hCom, SERVO_MOTOR_READ_BUFFER_SIZE, SERVO_MOTOR_WRITE_BUFFER_SIZE);

    // set timeout
    COMMTIMEOUTS TimeOuts; 
    TimeOuts.ReadIntervalTimeout = 1000;
    TimeOuts.ReadTotalTimeoutMultiplier = 0;
    TimeOuts.ReadTotalTimeoutConstant = 0;
    TimeOuts.WriteTotalTimeoutMultiplier = 0;
    TimeOuts.WriteTotalTimeoutConstant = 500;
    SetCommTimeouts(serial_port->hCom, &TimeOuts); 

    DCB dcb;
    GetCommState(serial_port->hCom, &dcb);
    dcb.BaudRate = baud_rate;
    switch (byte_size) {
        case 8: dcb.ByteSize = byte_size; break;
    }
    switch(parity) {
        case 0: dcb.Parity = NOPARITY; break;
        default: dcb.Parity = NOPARITY;
    }
    switch(stop_bits) {
        case 1: dcb.StopBits = ONESTOPBIT; break;
        case 2: dcb.StopBits = TWOSTOPBITS; break;
        default: dcb.StopBits = ONESTOPBIT;
    }
    if (!SetCommState(serial_port->hCom, &dcb))
        return -2;

    // clear the cache
    PurgeComm(serial_port->hCom, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
#else
    serial_port->hCom = open(com_id, O_RDWR | O_NOCTTY);
  	// Read in existing settings, and handle any error
    struct termios tty;
  	if (tcgetattr(serial_port->hCom, &tty) != 0) {
      	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      	return -3;
  	}
    switch(parity) {
        case 0: tty.c_cflag &= ~PARENB; break;
        default: tty.c_cflag &= ~PARENB;
    }
    switch(stop_bits) {
        case 1: tty.c_cflag &= ~CSTOPB; break;
        case 2: tty.c_cflag |= CSTOPB; break;
        default: tty.c_cflag &= ~CSTOPB;
    }    
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size 
    switch (byte_size) {
        case 8: tty.c_cflag |= CS8; break;
    }
  	tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;          // Disable echo
	tty.c_lflag &= ~ECHOE;         // Disable erasure
	tty.c_lflag &= ~ECHONL;        // Disable new-line echo
	tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST;         // Prevent special interpretation of output bytes (e.g. newline chars)

	tty.c_cc[VTIME] = 0;           // Wait for up to s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	tcflush(serial_port->hCom, TCIFLUSH);

	// other baud rate need to add
    switch (baud_rate) {
        case 2400:
            cfsetispeed(&tty, B2400);
            cfsetospeed(&tty, B2400);
            break;
        case 9600:
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);
            break;
        case 19200:
            cfsetispeed(&tty, B19200);
            cfsetospeed(&tty, B19200);
            break;
        case 38400:
            cfsetispeed(&tty, B38400);
            cfsetospeed(&tty, B38400);
            break;
        case 115200:
            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);
            break;
        default:
            printf("baud rate do not suport!\n");
            return -4;
    }

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port->hCom, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return -5;
	}
#endif
    serial_port->lock = 0;
    return 0;
}

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
int serial_write(serial* serial_port, DWORD write_size, BYTE write_buffer[]) 
{
    if (serial_port->lock) {
        printf("serial port is using!\n");
        return -1; //error
    }
    serial_port->lock = 1; // serial_port->lock the com 

#ifdef WIN32
    COMSTAT ComStat;
    DWORD dwErrorFlags;
    ClearCommError(serial_port->hCom, &dwErrorFlags, &ComStat);
#endif
    
#ifdef WIN32
    if (!WriteFile(
#else
    if (!write(
#endif
        serial_port->hCom, write_buffer, write_size
#ifdef WIN32
        , &write_size, NULL
#endif
    )) {
        printf("serial write failed!\n");
        serial_port->lock = 0;
        return -2;
    } else {
        serial_port->lock = 0;
        return 0;
    }
}

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
int serial_read(serial* serial_port, DWORD read_size, BYTE read_buffer[]) 
{
    if (serial_port->lock) {
        printf("serial port is using!\n");
        return -1; //error
    }
    serial_port->lock = 1; // serial_port->lock the com 

#ifdef WIN32
    COMSTAT ComStat;
    DWORD dwErrorFlags;
    ClearCommError(serial_port->hCom, &dwErrorFlags, &ComStat);
#endif

#ifdef WIN32
    if (!ReadFile(
#else
    if (!read(
#endif
        serial_port->hCom, read_buffer, read_size
#ifdef WIN32
        , &read_size, NULL
#endif
    )) {
        printf("serial read failed!\n");
        serial_port->lock = 0;
        return -2; 
    } else {
        serial_port->lock = 0;
        return 0; 
    }
}

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
void serial_crc16(BYTE data_packet[], int len, BYTE* CRC_L, BYTE* CRC_H)
{
    unsigned int i, pos, crc = 0xFFFF;
    for (pos = 0; pos < len; pos++) {
        crc ^= (unsigned int)data_packet[pos]; // XOR byte into least sig. byte of crc
        for (i = 8; i > 0; i--) {              // Loop over each bit
            if ((crc & 0x0001) != 0) {         // If the LSB is set
                crc >>= 1;                     // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                             // Else LSB is not set
                crc >>= 1;                     // Just shift right
        }
    }
    //crc = (((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8)); //Combined return
    *CRC_L = ((crc & 0x00ff));                
    *CRC_H = ((crc & 0xff00) >> 8);
}

/*
 * function: serial_close
 *     close the serial port
 * input:
 *     serial_port[serial *]: the serial struct pointor
 * output: 
 *     state[int]: success return 0
 */
int serial_close(serial* serial_port) 
{
#ifdef WIN32
    return CloseHandle(serial_port->hCom);
#else
    return close(serial_port->hCom);
#endif
}
