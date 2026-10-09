#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include "eyou_joints.h"
#include "polynomial_trajectory.h"
#include <unistd.h>
#include "config_loader.h"
int exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}

int main(int argc, char *argv[])
{
    ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.head_motor.ids, robot_config.hardware.head_motor.enabled);
    dual_arm.canable_init();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    ///////////////////////////////////////////////////////// Main Loop
    //---------------------------------------过程数据存储-----------------------------------------------------------
    int num_joints = 14;
    int step = 0;
    int data_size = 20000;
    int time_interval_us = robot_config.force_control.dt*1000000;
    //----------------------------------------------main loop-----------------------------------------------------
    while (!exit_requested &&step<data_size)
    {
        step++;
        printf("\n=== Step %d (Time: %.3fs) ===\n", 
                   step, step*time_interval_us/1000000.0f); 
        for (int i = 0;i<num_joints;++i)
        {
            if (!robot_config.hardware.head_motor.enabled[i])continue;
            dual_arm.set_torque(i, 0);
        }
        usleep(time_interval_us);
    }
    dual_arm.all_motor_stop();
    dual_arm.canable_close();
    return 0;
}
