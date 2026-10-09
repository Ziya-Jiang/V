#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>
#include <signal.h>

#include "cuarm_state.h"
#include "cuarm_message_handler.h"
#include "mcast_priority_server.h"


std::atomic<bool> stop_requested(false);
void handle_sig(int s) { 
    stop_requested = true; 
    std::cout << "Signal received, stopping..." << std::endl;
}

int main() {
    signal(SIGINT, handle_sig);

    char multicast_send_ip[] = "239.255.11.1";
    int  multicast_send_port = 25200;
    char receive_ip[] = "127.0.0.1";
    int  receive_port = 25201;

    McastPriorityServer<PanelCommand, PlannerState> server(
        receive_ip,
        receive_port,
        multicast_send_ip,
        multicast_send_port,
        &CuarmMessageHandler::unpack_panel_command,
        &CuarmMessageHandler::pack_planner_state,
        4096);

    PanelCommand panel_command;
    PlannerState planner_state;
    
    int cnt = 0;
    int arm_i = 0;
    while (!stop_requested){
        planner_state.send_timestamp = cnt;
        server.send(&planner_state);
        std::cout << "cnt: "<< cnt << " sent timestamp: " << planner_state.send_timestamp << std::endl;
        
        sleep_period(1000000);
        
        std::cout << "Current Client: " << server.getCurrentClientId() <<  ", priority: " << (int)server.getCurrentClientPriority() << std::endl;
        server.clearQueue(); //Clear old commands
        bool success = server.receive(1000);
        if (success){
            std::cout << "Queue: " << std::endl;
            server.printQueue();
            server.popQueue(&panel_command);
            std::cout   << "Selected cmd: \n"
                        << " priority: " << (int)panel_command.priority 
                        << " seq: " << panel_command.sequence 
                        << " Robot name: " << panel_command.RobotName[arm_i]
                        << " timestamp: " << panel_command.send_timestamp << std::endl;
        }
        cnt++;
        std::cout << "--------------------------------" << std::endl;
    }

    server.close();
    std::cout << "Mcast priority server closed\n";
    return 0;
}
