#if defined(_WIN32) || defined(WIN32)
    #define _CRT_SECURE_NO_WARNINGS
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
    #define msleep(ms) Sleep(ms)       // Windows expects ms directly
#else
    #include <unistd.h>
    #define msleep(ms) usleep(ms * 1000) // Linux converts ms to microseconds
#endif
#include <stdio.h>
#include <string.h>

#include "curi_tcp.h"

int main() {
    tcp_node sdk;
    int buf_size = 1024;

    printf("Connecting to Robot at 127.0.0.1:30001...\n");
    // is_server = 0. Connect to localhost.
    if (tcp_init(&sdk, (char*)"127.0.0.1", 30001, buf_size, 0) != 0) {
        printf("Connection Failed!\n");
        return -1;
    }

    // 1. Prepare and send a command
    strcpy(sdk.send_buffer, "MOVE_J 0,0,0");
    printf("Sending Move Command...\n");
    tcp_send(&sdk, buf_size);

    // 2. Wait for the robot to finish (Wait 1 seconds)
    int bytes = tcp_select(&sdk, 1000000, buf_size);
    if (bytes > 0) {
        printf("Robot Response: %s\n", sdk.receive_buffer);
    } else {
        printf("No response or timeout.\n");
    }

    printf("Going to Close.\n");
    tcp_close(&sdk);
    printf("Connection Closed.\n");
    return 0;
} 