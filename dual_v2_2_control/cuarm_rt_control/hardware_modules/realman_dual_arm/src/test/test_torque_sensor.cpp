#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include "robot_arm.h"
#include "config_loader.h"
#include "kinematics.h"
#include "can_fd.h"
#include "can_normal.h"
////////////////////////////////////////////////////////////// @brief Define pos control trajectory
bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}
int main(int argc, char *argv[])
{   
    signal(SIGINT, signal_handler);
    //Load configuration form config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robots = config_loader.getRobots();

    //Initialize CAN communication and pdo callback dispatcher
    canfd_init(robots[0].motors.can_channel, 0);
    canfd_init(robots[1].motors.can_channel, 0);

    can_normal_init(robots[0].torque_sensors.can_channel, EU_CANABLE);
    can_normal_init(robots[1].torque_sensors.can_channel, EU_CANABLE);
    //can_normal_init(robot_config.torque_sensors[1].can_channel, EU_CANABLE);
    std::ofstream fwrite_data("torque_sensor.txt");
    //Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    
    RobotArms arms[2];
    
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arms[i]) RobotArms(i, robots[i]);
        if (arms[i].initialize() != 0) return -1;
        if (arms[i].set_control_mode(pos_control) != 0) return -2;
    }
    double dt = 0.01;
    double act_cur[NUM_ARM][NUM_JOINTS_PER_ARM] = {0};
    double act_mdhpos[NUM_ARM][NUM_JOINTS_PER_ARM] = {0};
    double act_vel[NUM_ARM][NUM_JOINTS_PER_ARM] = {0};
    double act_tor[NUM_ARM][NUM_JOINTS_PER_ARM] = {0};
    while(!exit_requested)
    {
        auto clock_start = std::chrono::steady_clock::now();
        for (int i = 0;i<NUM_ARM;++i)
        {
            for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
            {
                if (arms[i].sensor_enable_list_[j]== 0) 
                    continue;
                arms[i].read_torque_sensor(j);
                act_tor[i][j] = arms[i].get_torque_sensor_data(j);
                arms[i].read_status(j);
                act_cur[i][j] = arms[i].get_current(j);
                std::cout << "arm: " << i << " joint: " << j
                          << " | act tor: " << act_tor[i][j]<< "| act cur: "<< act_cur[i][j] << std::endl;
                fwrite_data << act_tor[i][j] << " ";
            }
        }
        fwrite_data << std::endl;
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
        if (elapsed_us < dt * 1000000)
            std::this_thread::sleep_for(std::chrono::microseconds((long)(dt * 1000000 - elapsed_us)));
    }
    fwrite_data.close();
    return 0;
}
