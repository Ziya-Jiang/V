/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : curi_udp.h                                                 *
 * Author      : CHEN Wei                                                   *
 * Version     : 1.0.0                                                      *
 * Date        : 2022-04-29                                                 *
 * Description : Udp communication.                                         *
 * Others      : None                                                       *
 * History     : 2022-04-29 1st version.                                    *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/

#ifndef CURI_UDP_H
#define CURI_UDP_H

#if defined(_WIN32) || defined(WIN32)
	#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
	#endif
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#pragma comment(lib, "WS2_32.lib")
	typedef SOCKET curi_socket_t;
#else
	#include <netinet/in.h>
	#include <sys/select.h>
	typedef int curi_socket_t;
#endif
#include <stdint.h>

typedef struct _udp_node
{
	int CURI_SEND_PORT;
	int CURI_RECIVE_PORT;
	char* receive_buffer;
	char* send_buffer;
	int receive_size;
	int send_size;

	curi_socket_t receive_fd;
	curi_socket_t send_fd;

	struct sockaddr_in receive_addr;
	struct sockaddr_in send_addr;
	fd_set rset;
	uint8_t lock;
} udp_node;

#ifdef __cplusplus
extern "C"
{
#endif

int udp_init(udp_node* p, const char receive_ip[], int receive_port, const char send_ip[], int send_port, int buffer_size);
int udp_init1(udp_node* p, const char receive_ip[], int receive_port, const char send_ip[], int send_port, int buffer_size, curi_socket_t receive_fd);
int udp_init_receive_fd(udp_node* p, const char receive_ip[], int receive_port, int buffer_size);
int udp_init_receive_fd1(udp_node* p, const char receive_ip[], int receive_port, int buffer_size, curi_socket_t receive_fd);
int udp_init_send_fd(udp_node* p, const char send_ip[], int send_port, int buffer_size);

int udp_select(udp_node* p, int usec, int buffer_size);
void udp_send(udp_node* p, int buffer_size);
void udp_receive(udp_node* p, int buffer_size);
void udp_print(udp_node* p);
void udp_close(udp_node* p);

#ifdef __cplusplus
}
#endif
#endif
