#include "curi_tcp.h"

#if defined(_WIN32) || defined(WIN32)
	#include <ws2tcpip.h>
	#include <errno.h>
	#pragma comment(lib, "WS2_32.lib")
	#pragma comment(lib, "legacy_stdio_definitions.lib")
	#pragma comment(lib, "ucrt.lib")
	#define CURI_TCP_INVALID_FD INVALID_SOCKET
	#define CURI_TCP_IS_INVALID_FD(fd) ((fd) == INVALID_SOCKET)
	#define CURI_TCP_FAILED(ret) ((ret) == SOCKET_ERROR)
	#define CURI_TCP_CLOSE_FD(fd) closesocket(fd)
static int curi_tcp_wsa_startup(void)
{
	WSADATA wsaData;
	return WSAStartup(MAKEWORD(2, 2), &wsaData);
}

static void curi_tcp_wsa_cleanup(void)
{
	WSACleanup();
}
#else
	#include <arpa/inet.h>
	#include <sys/types.h>
	#include <unistd.h>
	#include <sys/time.h>
	#include <errno.h>
	#include <fcntl.h>
	#define CURI_TCP_INVALID_FD (-1)
	#define CURI_TCP_IS_INVALID_FD(fd) ((fd) < 0)
	#define CURI_TCP_FAILED(ret) ((ret) < 0)
	#define CURI_TCP_CLOSE_FD(fd) close(fd)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int curi_tcp_socket_errno(void)
{
#if defined(_WIN32) || defined(WIN32)
	return WSAGetLastError();
#else
	return errno;
#endif
}

static int curi_tcp_is_recv_would_block(int err)
{
#if defined(_WIN32) || defined(WIN32)
	return err == WSAEWOULDBLOCK || err == WSAEINTR;
#else
	return err == EAGAIN || err == EWOULDBLOCK || err == EINTR; 
#endif
}

static int curi_tcp_is_recv_disconnect(int err)
{
#if defined(_WIN32) || defined(WIN32)
	return err == WSAECONNRESET || err == WSAECONNABORTED ||
	       err == WSAENETRESET || err == WSAESHUTDOWN || err == WSAENOTCONN;
#else
	return err == ECONNRESET || err == EPIPE || err == ENOTCONN || err == ECONNABORTED;
#endif
}

static int curi_tcp_set_nonblocking(int fd, int enable)
{
#if defined(_WIN32) || defined(WIN32)
	u_long mode = enable ? 1UL : 0UL;
	return ioctlsocket((SOCKET)fd, FIONBIO, &mode);
#else
	int flags = fcntl(fd, F_GETFL, 0);
	if (flags < 0) return -1;
	return fcntl(fd, F_SETFL, enable ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK));
#endif
}

int tcp_init(tcp_node* p, const char* server_ip, int server_port, int buffer_size, bool is_server){
	int return_code = 0;
#if defined(_WIN32) || defined(WIN32)
	if (curi_tcp_wsa_startup() != 0) {
		return -1;
	}
#endif

	p->receive_buffer = (char*)malloc(buffer_size);
	p->send_buffer = (char*)malloc(buffer_size);
	if (!p->receive_buffer || !p->send_buffer) {
		return_code = -2;
		goto cleanup_all;
	}

	p->server_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (CURI_TCP_IS_INVALID_FD(p->server_fd)) {
		return_code = -3;
		goto cleanup_all;
	}

	memset(&(p->server_addr), 0, sizeof(p->server_addr));
	p->server_addr.sin_family = AF_INET;
	p->server_addr.sin_port = htons(server_port);

	if (is_server) {
		if ((server_ip && strlen(server_ip) > 0)){
			if (inet_pton(AF_INET, server_ip, &(p->server_addr.sin_addr)) <= 0) {
				fprintf(stderr, "inet_pton error: %d\n", curi_tcp_socket_errno());
				return_code = -4;
				goto cleanup_all;
			}
		}

		if (CURI_TCP_FAILED(bind(p->server_fd, (struct sockaddr*)&(p->server_addr), sizeof(p->server_addr)))) {
			fprintf(stderr, "bind error on ip %s and port %d: error %d\n",
				server_ip ? server_ip : "0.0.0.0", server_port, curi_tcp_socket_errno());
			return_code = -5;
			goto cleanup_all;
		}

		p->client_fd = CURI_TCP_INVALID_FD;
		listen(p->server_fd, 1);
	} else {
		if (!server_ip || inet_pton(AF_INET, server_ip, &(p->server_addr.sin_addr)) <= 0) {
			return_code = -6;
			goto cleanup_all;
		}

		if (CURI_TCP_FAILED(connect(p->server_fd, (struct sockaddr*)&p->server_addr, sizeof(p->server_addr)))) {
			fprintf(stderr, "connect error on ip %s and port %d: error %d\n",
				server_ip ? server_ip : "0.0.0.0", server_port, curi_tcp_socket_errno());
			return_code = -7;
			goto cleanup_all;
		}

		p->client_fd = p->server_fd;
		setsockopt(p->client_fd, SOL_SOCKET, SO_SNDBUF, (const char*)&buffer_size, sizeof(int));
	}
	return 0;

cleanup_all:
	if (!CURI_TCP_IS_INVALID_FD(p->server_fd)) {
		CURI_TCP_CLOSE_FD(p->server_fd);
		p->server_fd = CURI_TCP_INVALID_FD;
	}
	if (p->receive_buffer) free(p->receive_buffer);
	if (p->send_buffer) free(p->send_buffer);
	p->receive_buffer = p->send_buffer = NULL;

	return return_code;
}

int tcp_server_wait_client(tcp_node* p, int timeout_usec, int buffer_size){
	if (CURI_TCP_IS_INVALID_FD(p->server_fd) ||
		!CURI_TCP_IS_INVALID_FD(p->client_fd) ||
		p->server_fd == p->client_fd) {
		return -1;
	}

	FD_ZERO(&(p->rset));
	FD_SET(p->server_fd, &(p->rset));

	struct timeval t;
	struct timeval* t_ptr = NULL;
	if (timeout_usec >= 0){
		t.tv_sec = timeout_usec / 1000000;
		t.tv_usec = timeout_usec % 1000000;
		t_ptr = &t;
	}

	int nready = select((int)p->server_fd + 1, &(p->rset), NULL, NULL, t_ptr);

	if (nready > 0){
#if defined(_WIN32) || defined(WIN32)
		int addr_len = (int)sizeof(p->client_addr);
#else
		socklen_t addr_len = sizeof(p->client_addr);
#endif
		p->client_fd = accept(p->server_fd, (struct sockaddr *)&p->client_addr, &addr_len);
		if (CURI_TCP_IS_INVALID_FD(p->client_fd)){
			return -2;
		}

		char current_byte;
		int bytes;
		int recv_err = 0;
#if defined(_WIN32) || defined(WIN32)
		if (curi_tcp_set_nonblocking((int)p->client_fd, 1) != 0) {
			return -3;
		}
		while ((bytes = recv(p->client_fd, &current_byte, 1, 0)) > 0);
		if (CURI_TCP_FAILED(bytes)) recv_err = curi_tcp_socket_errno();
		curi_tcp_set_nonblocking((int)p->client_fd, 0);
#else
		while ((bytes = (int)recv(p->client_fd, &current_byte, 1, MSG_DONTWAIT)) > 0);
		if (CURI_TCP_FAILED(bytes)) recv_err = curi_tcp_socket_errno();
#endif

		if (bytes == 0 || (CURI_TCP_FAILED(bytes) && curi_tcp_is_recv_disconnect(recv_err))) {
			tcp_server_clear_client(p);
			return tcp_server_wait_client(p, timeout_usec, buffer_size);
		} else if (CURI_TCP_FAILED(bytes) && curi_tcp_is_recv_would_block(recv_err)) {
			setsockopt(p->client_fd, SOL_SOCKET, SO_SNDBUF, (const char*)&buffer_size, sizeof(int));
			return 0;
		} else {
			return -3;
		}
	} else if (nready < 0){
		return -4;
	}
	return 1;
}

void tcp_server_clear_client(tcp_node* p){
	if (!CURI_TCP_IS_INVALID_FD(p->client_fd) && p->client_fd != p->server_fd) {
		CURI_TCP_CLOSE_FD(p->client_fd);
		p->client_fd = CURI_TCP_INVALID_FD;
	}
}

int tcp_server_has_client(tcp_node* p){
	return !CURI_TCP_IS_INVALID_FD(p->client_fd) && p->client_fd != p->server_fd;
}

int tcp_server_get_client_info(tcp_node* p, char ip[], int port){
	if (!tcp_server_has_client(p)) return -1;
	inet_ntop(AF_INET, &(p->client_addr.sin_addr), ip, sizeof(ip));
	port = ntohs(p->client_addr.sin_port);
	return 0;
}

static int tcp_probe_client_disconnect(tcp_node* p)
{
	char ch;
	int probe_err = 0;
	int probe;

#if defined(_WIN32) || defined(WIN32)
	if (curi_tcp_set_nonblocking((int)p->client_fd, 1) != 0) {
		return 0;
	}
	probe = recv(p->client_fd, &ch, 1, MSG_PEEK);
	if (CURI_TCP_FAILED(probe)) probe_err = curi_tcp_socket_errno();
	curi_tcp_set_nonblocking((int)p->client_fd, 0);
#else
	probe = (int)recv(p->client_fd, &ch, 1, MSG_PEEK | MSG_DONTWAIT);
	if (CURI_TCP_FAILED(probe)) probe_err = curi_tcp_socket_errno();
#endif

	if (probe == 0) {
		return 1;
	}
	if (CURI_TCP_FAILED(probe) && !curi_tcp_is_recv_would_block(probe_err)) {
		return 1;
	}
	return 0;
}

int tcp_select(tcp_node* p, int timeout_usec, int buffer_size){
	if (CURI_TCP_IS_INVALID_FD(p->client_fd)) {
		return -3;
	}

	FD_ZERO(&(p->rset));
	FD_SET(p->client_fd, &(p->rset));

	struct timeval t;
	t.tv_sec = timeout_usec / 1000000;
	t.tv_usec = timeout_usec % 1000000;

	int nready = select((int)p->client_fd + 1, &(p->rset), NULL, NULL, &t);

	p->receive_size = 0;
	if (nready > 0 && FD_ISSET(p->client_fd, &(p->rset))) {
		memset(p->receive_buffer, 0, buffer_size);
		int bytes = recv(p->client_fd, p->receive_buffer, buffer_size, 0);
		if (bytes > 0) {
			p->receive_size = bytes;
			return bytes; //bytes > 0 data arrived successfully.
		}
		return -1; //bytes < 0 clean disconnect
	} else if (nready < 0) {
		return -3; //An internal socket error occurred
	}

	if (tcp_probe_client_disconnect(p)) {
		return -1; //Disconnect or error caught
	}
	return -4; //Timeout reached
}

void tcp_send(tcp_node* p, int buffer_size){
	(void)buffer_size;
	if (CURI_TCP_IS_INVALID_FD(p->client_fd)) return;

	int sent = send(p->client_fd, p->send_buffer, (int)strlen(p->send_buffer), 0);
	if (CURI_TCP_FAILED(sent)) {
		perror("TCP Send failed");
	}
}

int tcp_receive(tcp_node* p, int buffer_size){
	if (CURI_TCP_IS_INVALID_FD(p->client_fd)) return -1;

	memset(p->receive_buffer, 0, buffer_size);

	p->receive_size = recv(p->client_fd, p->receive_buffer, buffer_size - 1, 0);

	if (p->receive_size == 0)
		return 0;
	else if (CURI_TCP_FAILED(p->receive_size))
		return -1;

	return p->receive_size;
}

void tcp_print(tcp_node* p, int buffer_size){
	if (p->receive_size > 0) {
		int term_index = (p->receive_size >= buffer_size) ? buffer_size - 1 : p->receive_size;
		p->receive_buffer[term_index] = '\0';
		puts(p->receive_buffer);
	}
}

void tcp_close(tcp_node* p){
	if (p->receive_buffer) { free(p->receive_buffer); p->receive_buffer = NULL; }
	if (p->send_buffer) { free(p->send_buffer); p->send_buffer = NULL; }

	if (!CURI_TCP_IS_INVALID_FD(p->client_fd) && p->client_fd != p->server_fd) {
		CURI_TCP_CLOSE_FD(p->client_fd);
		p->client_fd = CURI_TCP_INVALID_FD;
	}
	if (!CURI_TCP_IS_INVALID_FD(p->server_fd)) {
		CURI_TCP_CLOSE_FD(p->server_fd);
		p->server_fd = CURI_TCP_INVALID_FD;
	}

#if defined(_WIN32) || defined(WIN32)
	curi_tcp_wsa_cleanup();
#endif
}
