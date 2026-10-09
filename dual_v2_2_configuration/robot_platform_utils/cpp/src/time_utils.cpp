#include <time_utils.h>
#include <time.h>

#if defined(_WIN32) || defined(WIN32)
    // winsock2.h MUST come before windows.h to correctly define struct timeval
    #include <winsock2.h> 
    #include <windows.h>
    #include <chrono>
    #include <thread>

    // Drop-in replacement for gettimeofday using modern standard C++
    inline int gettimeofday(struct timeval* tp, void* tzp) {
        if (tp) {
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
            auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration) - seconds;
            
            tp->tv_sec = static_cast<long>(seconds.count());
            tp->tv_usec = static_cast<long>(microseconds.count());
        }
        return 0;
    }
#else
    #include <sys/time.h>
#endif

void sleep_period(int sleep_time_in_us)
{
    if (sleep_time_in_us < 0)
        return;
#if defined(_WIN32) || defined(WIN32)
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_in_us));
#else
    struct timespec ts;
    ts.tv_sec = sleep_time_in_us / 1000000L;
    ts.tv_nsec = (sleep_time_in_us % 1000000L) * 1000L;
    clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
#endif
}

long get_time_now(){
    #ifdef KERNEL_XENOMAI
        RTIME now = rt_timer_read();
        return now / 1000;
    #else // preempt_rt, linux & windows
        struct timeval t;
        gettimeofday(&t, NULL);
        return t.tv_sec * 1000000 + t.tv_usec;
    #endif
}
