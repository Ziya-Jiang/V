#include "curi_tcp.h"
#include <stdbool.h>

int main() {
    tcp_node robot;
    int buf_size = 1024;

    printf("Initializing Server.\n");
    // is_server = 1. Listen on port 30001. 
    // Passing NULL for IP listens on all interfaces (INADDR_ANY).
    if (tcp_init(&robot, NULL, 30001, buf_size, true) != 0) {
        printf("Init Failed!\n");
        return -1;
    }
    printf("Waiting for Client...\n");
    tcp_server_wait_client(&robot, -1, buf_size); //-1 timeout to wait until a client connects
    printf("Client connected. Waiting for commands...\n");

    while (true) {
        // Use select to check for data with a 1-second timeout
        int bytes = tcp_select(&robot, 1000000, buf_size);

        if (bytes > 0) {
            printf("Executing: %s\n", robot.receive_buffer);
            
            // Send feedback
            strcpy(robot.send_buffer, "COMMAND_DONE");
            tcp_send(&robot, buf_size);
        } else if (bytes <= 0 && bytes != -4) {
            printf("SDK Disconnected.\n");
            break;
        }
    }

    tcp_close(&robot);
    return 0;
}