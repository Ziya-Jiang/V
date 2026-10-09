/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : curi_udp.c                                                 *
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
#include "curi_udp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32) || defined(WIN32)
	#ifndef CURI_UDP_INVALID_FD
	#define CURI_UDP_INVALID_FD INVALID_SOCKET
	#endif
	#define CURI_UDP_IS_INVALID_FD(fd) ((fd) == INVALID_SOCKET)
	#define CURI_UDP_FAILED(ret) ((ret) == SOCKET_ERROR)
	#define CURI_UDP_CLOSE_FD(fd) closesocket(fd)
	static int g_wsa_usage_count = 0;
#else
	#ifndef CURI_UDP_INVALID_FD
	#define CURI_UDP_INVALID_FD (-1)
	#endif
	#define CURI_UDP_IS_INVALID_FD(fd) ((fd) < 0)
	#define CURI_UDP_FAILED(ret) ((ret) < 0)
	#define CURI_UDP_CLOSE_FD(fd) close(fd)
	#include <arpa/inet.h>
	#include <sys/socket.h>
	#include <sys/types.h>
	#include <unistd.h>
	#include <sys/time.h>
	#include <errno.h>
#endif

static int curi_udp_wsa_startup(void)
{
#if defined(_WIN32) || defined(WIN32)
	if (g_wsa_usage_count == 0) {
		WSADATA wsaData;
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
			return -1;
		}
	}
	g_wsa_usage_count++;
#endif
	return 0;
}

static void curi_udp_wsa_cleanup(void)
{
#if defined(_WIN32) || defined(WIN32)
	if (g_wsa_usage_count > 0) {
		g_wsa_usage_count--;
		if (g_wsa_usage_count == 0) {
			WSACleanup();
		}
	}
#endif
}

static int curi_udp_socket_errno(void)
{
#if defined(_WIN32) || defined(WIN32)
	return WSAGetLastError();
#else
	return errno;
#endif
}

int udp_init(udp_node* p, const char receive_ip[], int receive_port, const char send_ip[], int send_port, int buffer_size)
{
	if (!p) return -1;
	p->receive_buffer = NULL;
	p->send_buffer = NULL;
	p->receive_fd = CURI_UDP_INVALID_FD;
	p->send_fd = CURI_UDP_INVALID_FD;
	if (curi_udp_wsa_startup() != 0) {
		return -1;
	}

#if defined(_WIN32) || defined(WIN32)
	SOCKET rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
#else
	int rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
#endif

	int ret = udp_init_receive_fd1(p, receive_ip, receive_port, buffer_size, rx_fd);
	if (ret != 0) {
		printf("Failed to initialize receive_fd, return code: %d\n", ret);
		udp_close(p);
		return -1;
	}

	ret = udp_init_send_fd(p, send_ip, send_port, buffer_size);
	if (ret != 0) {
		printf("Failed to initialize send_fd, return code: %d\n", ret);
		udp_close(p);
		return -1;
	}
	return 0;
}

int udp_init1(udp_node* p, const char receive_ip[], int receive_port, const char send_ip[], int send_port, int buffer_size, curi_socket_t receive_fd)
{
	if (!p) return -1;
	p->receive_buffer = NULL;
	p->send_buffer = NULL;
	p->receive_fd = CURI_UDP_INVALID_FD;
	p->send_fd = CURI_UDP_INVALID_FD;

	if (curi_udp_wsa_startup() != 0) {
		return -1;
	}

	int ret = udp_init_receive_fd1(p, receive_ip, receive_port, buffer_size, receive_fd);
	if (ret != 0) {
		printf("Failed to initialize receive_fd, return code: %d\n", ret);
		udp_close(p);
		return -1;
	}

	ret = udp_init_send_fd(p, send_ip, send_port, buffer_size);
	if (ret != 0) {
		printf("Failed to initialize send_fd, return code: %d\n", ret);
		udp_close(p);
		return -1;
	}
	return 0;
}

int udp_init_receive_fd(udp_node* p, const char receive_ip[], int receive_port, int buffer_size)
{
	if (!p) return -1;
	p->receive_buffer = NULL;
	p->send_buffer = NULL;
	p->receive_fd = CURI_UDP_INVALID_FD;
	p->send_fd = CURI_UDP_INVALID_FD;

	if (curi_udp_wsa_startup() != 0) {
		return -1;
	}

#if defined(_WIN32) || defined(WIN32)
	SOCKET rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
#else
	int rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
#endif

	int ret = udp_init_receive_fd1(p, receive_ip, receive_port, buffer_size, rx_fd);
	if (ret != 0) {
		udp_close(p);
	}
	return ret;
}

int udp_init_receive_fd1(udp_node* p, const char receive_ip[], int receive_port, int buffer_size, curi_socket_t receive_fd)
{
	if (!p) return -1;
	
	p->lock = 0;
#if defined(_WIN32) || defined(WIN32)
	p->receive_fd = (SOCKET)receive_fd;
#else
	p->receive_fd = receive_fd;
#endif

	if (CURI_UDP_IS_INVALID_FD(p->receive_fd)) {
		fprintf(stderr, "socket create error: %d\n", curi_udp_socket_errno());
		return -4;
	}

	p->CURI_RECIVE_PORT = receive_port;

	memset(&(p->receive_addr), 0, sizeof(p->receive_addr));
	p->receive_addr.sin_port = htons(receive_port);
	p->receive_addr.sin_family = AF_INET;

	if (inet_pton(AF_INET, receive_ip, &(p->receive_addr.sin_addr)) <= 0) {
		fprintf(stderr, "Invalid receive IP address format\n");
		return -1;
	}

	if (CURI_UDP_FAILED(bind(p->receive_fd, (const struct sockaddr *)&(p->receive_addr), sizeof(p->receive_addr)))) {
		fprintf(stderr, "bind error: %d\n", curi_udp_socket_errno());
		return -2;
	}

	p->receive_buffer = (char*)malloc(buffer_size);
	if (!p->receive_buffer) {
		return -3;
	}

	if (0 != setsockopt(p->receive_fd, SOL_SOCKET, SO_RCVBUF, (const char*)&buffer_size, sizeof(int))) {
		return -4;
	}

	FD_ZERO(&(p->rset));
	return 0;
}

int udp_init_send_fd(udp_node* p, const char send_ip[], int send_port, int buffer_size)
{
	if (!p) return -1;

	p->lock = 0;
	p->send_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (CURI_UDP_IS_INVALID_FD(p->send_fd)) {
		return -2;
	}

	p->CURI_SEND_PORT = send_port;

	memset(&(p->send_addr), 0, sizeof(p->send_addr));
	p->send_addr.sin_port = htons(send_port);
	p->send_addr.sin_family = AF_INET;

	if (inet_pton(AF_INET, send_ip, &(p->send_addr.sin_addr)) <= 0) {
		fprintf(stderr, "Invalid send IP address format\n");
		return -3;
	}

	p->send_buffer = (char*)malloc(buffer_size);
	if (!p->send_buffer) {
		return -4;
	}

	if (0 != setsockopt(p->send_fd, SOL_SOCKET, SO_SNDBUF, (const char*)&buffer_size, sizeof(int))) {
		return -5;
	}

	FD_ZERO(&(p->rset));
	return 0;
}

int udp_select(udp_node* p, int usec, int buffer_size)
{
	if (!p || CURI_UDP_IS_INVALID_FD(p->receive_fd) || !p->receive_buffer) {
		return 0;
	}

	FD_ZERO(&(p->rset));
	FD_SET(p->receive_fd, &(p->rset));

	struct timeval t;
	t.tv_sec = usec / 1000000;
	t.tv_usec = usec % 1000000;

	// Cast select fd parameter safely for platform variances
	int nready = select((int)p->receive_fd + 1, &(p->rset), NULL, NULL, &t);
	if (nready > 0 && FD_ISSET(p->receive_fd, &(p->rset))) {
		struct sockaddr_in sender_addr;
		memset(&sender_addr, 0, sizeof(sender_addr));
#if defined(_WIN32) || defined(WIN32)
		int sender_addr_len = (int)sizeof(sender_addr);
#else
		socklen_t sender_addr_len = sizeof(sender_addr);
#endif
		p->receive_size = recvfrom(
			p->receive_fd,
			p->receive_buffer,
			buffer_size > 0 ? buffer_size - 1 : 0,
			0,
			(struct sockaddr*)&sender_addr,
			&sender_addr_len
		);
		if (p->receive_size < 0) {
			p->receive_size = 0;
		} else if (sender_addr.sin_addr.s_addr != p->send_addr.sin_addr.s_addr) {
			fprintf(stderr, "Rejected UDP datagram from unexpected peer\n");
			p->receive_size = 0;
		} else {
			p->receive_buffer[p->receive_size] = '\0';
		}
	} else {
		p->receive_size = 0;
	}
	return p->receive_size;
}

void udp_send(udp_node* p, int buffer_size)
{
	if (!p || CURI_UDP_IS_INVALID_FD(p->send_fd) || !p->send_buffer) {
		return;
	}
	sendto(
		p->send_fd,
		p->send_buffer,
		buffer_size,
		0,
		(const struct sockaddr *)&(p->send_addr),
		sizeof(p->send_addr)
	);
}

void udp_receive(udp_node* p, int buffer_size)
{
	if (!p || CURI_UDP_IS_INVALID_FD(p->receive_fd) || !p->receive_buffer) {
		if (p) p->receive_size = 0;
		return;
	}
	p->receive_size = recv(p->receive_fd, p->receive_buffer, buffer_size, 0);
	if (p->receive_size < 0) {
		p->receive_size = 0;
	}
}

void udp_print(udp_node* p)
{
	if (!p || !p->receive_buffer || p->receive_size <= 0) {
		return;
	}
	fwrite(p->receive_buffer, 1, p->receive_size, stdout);
	putchar('\n');
}

void udp_close(udp_node* p)
{
	if (!p) {
		return;
	}

	if (p->receive_buffer) {
		free(p->receive_buffer);
		p->receive_buffer = NULL;
	}
	if (p->send_buffer) {
		free(p->send_buffer);
		p->send_buffer = NULL;
	}

	if (!CURI_UDP_IS_INVALID_FD(p->receive_fd)) {
		CURI_UDP_CLOSE_FD(p->receive_fd);
		p->receive_fd = CURI_UDP_INVALID_FD;
		curi_udp_wsa_cleanup();
	}
	if (!CURI_UDP_IS_INVALID_FD(p->send_fd)) {
		CURI_UDP_CLOSE_FD(p->send_fd);
		p->send_fd = CURI_UDP_INVALID_FD;
		curi_udp_wsa_cleanup();
	}
}