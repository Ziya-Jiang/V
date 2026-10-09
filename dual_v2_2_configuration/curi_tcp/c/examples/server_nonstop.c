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

    while (true) {
        if (tcp_server_has_client(&robot) == 0){ 
            printf("Waiting for Client...\n");
            int ret = tcp_server_wait_client(&robot, -1, buf_size); //-1 timeout to wait until a client connects

            if (ret < 0) {
                printf("Error waiting for client: %d\n", ret);
                break;
            }else if (ret == 0){
                printf("client addr: %s:%d\n",
                    inet_ntoa(robot.client_addr.sin_addr),
                    ntohs(robot.client_addr.sin_port));
                printf("Client connected. Waiting for commands...\n");
            }
        }else{
            // Use select to check for data with a 1-second timeout
            int bytes = tcp_select(&robot, 1000000, buf_size);

            if (bytes > 0) {
                printf("Executing: %s\n", robot.receive_buffer);
                
                // Send feedback
                strcpy(robot.send_buffer, "COMMAND_DONE");
                tcp_send(&robot, buf_size);
            } else if (bytes <= 0 && bytes != -4) {
                printf("Client disconnected.\n");
                tcp_server_clear_client(&robot);
            }
        }
    }

    tcp_close(&robot);
    return 0;
}