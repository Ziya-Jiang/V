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
#include <sys/time.h>   // defines struct timeval

/*
 * function: udp_init
 *     udp communication initialization
 * input:
 *     p[udp_node *]: the udp_node structure
 *     local_ip[char *]: the local node ip
 *     local_port[int]: data send port
 *     remote_port[int]: receive command port
 * output:
 *     state[int]: success return 0
 */
int udp_init(udp_node* p, char local_ip[], int local_port, char remote_ip[], int remote_port, int buffer_size)
{
#ifdef WIN32
	WSADATA wsaData;
	int err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0) {
		return err;
	}

	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
		WSACleanup();
		return -1;
	}
#endif 
    p->lock = 0;
	// create UDP socket 
	p->server_fd = socket(AF_INET, SOCK_DGRAM, 0);
	p->server_addr.sin_addr.s_addr = inet_addr(local_ip);
	p->server_addr.sin_port = htons(local_port);
	p->server_addr.sin_family = AF_INET;

	p->client_fd = socket(AF_INET, SOCK_DGRAM, 0);
	p->client_addr.sin_addr.s_addr = inet_addr(remote_ip);
	p->client_addr.sin_port = htons(remote_port);
	p->client_addr.sin_family = AF_INET;

	// bind server address to socket descriptor 
#ifdef WIN32
	if (bind(p->server_fd, (SOCKADDR*)&(p->server_addr)), sizeof(p->server_addr)) == -1) {
#else
	if (bind(p->server_fd, (const struct sockaddr *)&(p->server_addr), sizeof(p->server_addr)) == -1) {
#endif
		perror("bind error.");
		return -1;
	}

	// set the buffer size
	p->receive_buffer = (char*)malloc(buffer_size);
	p->send_buffer = (char*)malloc(buffer_size);
	if (!p->receive_buffer || !p->send_buffer) {
		free(p->receive_buffer); 
		free(p->send_buffer); 
		return -2; 
	}

	// set the buffer size for udp socket
	if (0 != setsockopt(p->server_fd, SOL_SOCKET, SO_SNDBUF, (const char*)&buffer_size, sizeof(int)))
		return -3;
	else if (0 != setsockopt(p->client_fd, SOL_SOCKET, SO_RCVBUF, (const char*)&buffer_size, sizeof(int)))
		return -4;
	//else if (0 != setsockopt(p->client_fd, SOL_SOCKET, SO_REUSEADDR, 0, sizeof(int)))
	//	return -4;

	FD_ZERO(&(p->rset));

	return 0;
}

/*
 * function: udp_select
 *     udp communication selection to test if there are some data at the port
 * input:
 *     p[udp_node *]: the udp_node structure
 *     usec[int]: wait data time in us
 * output:
 *     [int]: the data received
 */
int udp_select(udp_node* p, int usec, int buffer_size)
{
	FD_SET(p->server_fd, &(p->rset));
	// select the ready descriptor 
	struct timeval t;
	t.tv_sec = 0;
	t.tv_usec = usec;
	int nready = select(p->server_fd + 1, &(p->rset), NULL, NULL, &t);
	if (FD_ISSET(p->server_fd, &(p->rset))) {
		memset(p->receive_buffer, 0, buffer_size);
		p->receive_size = recv(p->server_fd, p->receive_buffer, buffer_size, 0);
	} else {
		p->receive_size = 0;
	}
	return p->receive_size;
}

/*
 * function: udp_send
 *     udp communication send data to the remote port
 * input:
 *     p[udp_node *]: the udp_node structure
 * output:
 *     [void]
 */
void udp_send(udp_node* p, int buffer_size)
{
	// lock_udp_node(p, 1);
	char send_buffer[buffer_size];
	strncpy(send_buffer, p->send_buffer, strlen(p->send_buffer));
	sendto(p->client_fd, (struct sockaddr *)&(send_buffer), strlen(send_buffer), 0,
		(const struct sockaddr *)&(p->client_addr), sizeof(p->client_addr));
	// lock_udp_node(p, 0);
}

/*
 * function: udp_receive
 *     udp communication receive data from the local port
 * input:
 *     p[udp_node *]: the udp_node structure
 * output:
 *     [void]
 */
void udp_receive(udp_node* p, int buffer_size)
{
	p->receive_size = recv(p->server_fd, p->receive_buffer, buffer_size, 0);
}

/*
 * function: udp_print
 *     udp communication print the receive data from the local port
 * input:
 *     p[udp_node *]: the udp_node structure
 * output:
 *     [void]
 */
void udp_print(udp_node* p)
{
	p->receive_buffer[p->receive_size] = 0;
	puts(p->receive_buffer);
}

/*
 * function: udp_close
 *     udp communication close the port
 * input:
 *     p[udp_node *]: the udp_node structure
 * output:
 *     [void]
 */
void udp_close(udp_node* p)
{
	free(p->receive_buffer);
	free(p->send_buffer);
	p->receive_buffer = NULL;
	p->send_buffer = NULL;
#ifdef WIN32
	closesocket(p->server_fd);
	closesocket(p->client_fd);
	WSACleanup();
#else
	close(p->server_fd);
	close(p->client_fd);
#endif
}
