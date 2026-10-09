#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#include <unistd.h>
#include <chrono>
#include <mutex>
#include "eyou_joints.h"
#include "cuarm_udp.h"
#include "cuarm_state.h"
#include "kinematics.h"
#include "config_loader.h"

bool exit_requested = false;
std::mutex receive_udp_mtx;
bool shutdown_all = false;
DualArmState received_command;
DualArmState send_status;
udp_node udp;
ConfigLoader robot_config(std::string (PROJECT_SOURCE_DIR)+std::string("/config.yaml"));
EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
double joint_pos[NUM_JOINTS]={0.}, mdh_rad[2][7]={0.}, ee_pos_ori[2][6]={0.};

void signal_handler(int signal)
{
    exit_requested = true;
    shutdown_all = true;
}

void receive_udp_task()
{
    auto t0 = std::chrono::steady_clock::now();
    int dt_ms = 300;

    DualArmState temp;
    temp.JointSize = 0;
    while (!shutdown_all)
    {
        if (udp_select(&udp, 50))
        {
            udp_unpack_dual_arm_state(&udp, &temp);
        }
        {
            std::lock_guard<std::mutex> lock(receive_udp_mtx);
            copy_dual_arm_command(&temp, &received_command);
        }
        auto t1 = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        if(elapsed < dt_ms)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms - elapsed));
        }
    }
    std::cout << "Exit receive_udp_task" << std::endl;
}


int main()
{
    signal(SIGINT, signal_handler);
    udp_init(&udp, (char *)"192.168.3.95", (char *)"192.168.3.42", 10086, 10085);
    
    std::thread receive_udp_thread(receive_udp_task);
    while(!exit_requested)
    {
        DualArmState local_copy;
        {
            std::lock_guard<std::mutex> lock(receive_udp_mtx);
            copy_dual_arm_command(&received_command, &local_copy);
        }
        
        // 使用本地副本打印，不持有锁
        if (local_copy.JointSize > 0) {
            print_dual_arm_state(&local_copy);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    receive_udp_thread.join();
    return 0;
}