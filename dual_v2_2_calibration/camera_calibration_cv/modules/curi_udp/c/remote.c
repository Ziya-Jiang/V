#include "curi_udp.h"

long get_time_now()
{
#ifdef KERNEL_XENOMAI
    RTIME now = rt_timer_read();
    return now / 1000;
#else // preempt_rt & linux
    struct timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec * 1000000 + t.tv_usec;
#endif
}

void udp_pack(udp_node* p, long t)
{
	int i;
	for (i = 0; i < 100; ++ i)
		sprintf(&p->send_buffer[i], "%ld", t);
}

void udp_unpack(udp_node* p, long *t)
{
	int i;
	for (i = 0; i < 100; ++ i)
		sscanf(&p->recieve_buffer[i], "%ld", t);
}

int main()
{
	udp_node comunication1;
	if (!udp_init(&comunication1, "127.0.0.1", 13331, 13332)) {
		printf("comunication1 create.\n");
	}
	else {
		printf("comunication1 create failure!\n");
	}

	long t0, t1;
	for (int i = 0; i < 10000; ++ i) {
		if (i % 5 == 0) {
			t0 = get_time_now();
			udp_pack(&comunication1, t0);
			udp_send(&comunication1);
		}
		usleep(2000);
		if (udp_select(&comunication1, 50)) {
			udp_unpack(&comunication1, &t1);
			printf("t diff = %ld\n", t1 - t0);
		}
	}

	udp_close(&comunication1);

	return 0;
}