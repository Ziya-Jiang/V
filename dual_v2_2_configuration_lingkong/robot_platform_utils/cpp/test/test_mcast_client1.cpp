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
#include "mcast_client.h"


std::atomic<bool> stop_requested(false);
void handle_sig(int s) { 
    stop_requested = true; 
    std::cout << "Signal received, stopping..." << std::endl;
}

int main() {
    signal(SIGINT, handle_sig);

    char multicast_receive_ip[] = "239.255.11.1";
    int  multicast_receive_port = 25200;
    char send_ip[] = "127.0.0.1";
    int  send_port = 25201;

    McastClient<PlannerState, PanelCommand> client(
        multicast_receive_ip,
        multicast_receive_port,
        send_ip,
        send_port,
        &CuarmMessageHandler::unpack_planner_state,
        &CuarmMessageHandler::pack_panel_command,
        4096);
    PlannerState planner_state;
    PanelCommand panel_command;
    panel_command.priority = CommandPriorityLevel::kOther;
    panel_command.sequence = 0;
    panel_command.send_timestamp = 0;
    panel_command.ArmSize = 1;
    panel_command.JointSize[0] = 6;
    panel_command.ArmWaypointSize = 0;
    panel_command.GripperSize = 1;
    panel_command.GripperJointSize[0] = 1;
    std::string str = "CLIENT_1";
    strcpy(panel_command.RobotName[0], str.c_str());

    int cnt = 0;
    while (!stop_requested){
        bool success = client.receive(&planner_state, 10000);
        if (success){
            std::cout << "Client received from server: " << planner_state.send_timestamp << std::endl;

            std::cout << "Sending panel commands, timestamp: " << panel_command.send_timestamp << std::endl;
            client.send(&panel_command);

            panel_command.send_timestamp++;
            panel_command.sequence++;
            std::cout << "--------------------------------" << std::endl;
        }
        cnt++;
        sleep_period(1200000);
    }

    client.close();
    std::cout << "Mcast clients closed" << std::endl;
    return 0;
}
