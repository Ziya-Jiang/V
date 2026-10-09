#include "curi_udp.h"
#include <stdio.h> 

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif

#define UDP_BUFFER_SIZE 4096

static void sleep_usec(int usec)
{
#if defined(_WIN32) || defined(WIN32)
	Sleep((DWORD)((usec + 999) / 1000));
#else
	usleep(usec);
#endif
}

long get_time_now()
{
#ifdef KERNEL_XENOMAI
	RTIME now = rt_timer_read();
	return now / 1000;
#elif defined(_WIN32) || defined(WIN32)
	LARGE_INTEGER freq, counter;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&counter);
	return (long)((counter.QuadPart * 1000000) / freq.QuadPart);
#else
	struct timeval t;
	gettimeofday(&t, NULL);
	return t.tv_sec * 1000000 + t.tv_usec;
#endif
}

void udp_pack(udp_node* p, int index, long t)
{
	snprintf(p->send_buffer, UDP_BUFFER_SIZE, "%d#%ld", index, t);
}

void udp_unpack(udp_node* p, int *index, long *t)
{
    if (p->receive_buffer[0] != '\0') {
        sscanf(p->receive_buffer, "%d#%ld", index, t);
    }
}

int main()
{
	udp_node comunication1;
	if (!udp_init(&comunication1, "127.0.0.1", 13332, "127.0.0.1", 13331, UDP_BUFFER_SIZE)) {
		printf("comunication1 create.\n");
	}
	else {
		printf("comunication1 create failure!\n");
	}

	int received_index = 0, send_index = 0;
	long received_data = 0, send_data = 0;
	for (int i = 0; i < 10000; ++ i) {
		if (udp_select(&comunication1, 50, UDP_BUFFER_SIZE)) {
			udp_unpack(&comunication1, &received_index, &received_data);
		}
		
		if (i % 5 == 0) {
			send_data = get_time_now();
			send_index++;
			udp_pack(&comunication1, send_index, send_data);
			udp_send(&comunication1, UDP_BUFFER_SIZE);
		}
		printf("Local: received index: %d, data: %ld, sent index: %d, data: %ld\n", received_index, received_data, send_index, send_data);
		sleep_usec(2000);
	}

	udp_close(&comunication1);

	return 0;
}
