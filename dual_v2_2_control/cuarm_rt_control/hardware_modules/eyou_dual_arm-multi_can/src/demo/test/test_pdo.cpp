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
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
#include "inspire_hand.h"
////////////////////////////////////////////////////////////// @brief Define pos control trajectory
int main(int argc, char *argv[])
{
   //Load configuration form config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();
    // Initialize CAN communication and pdo callback dispatcher
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    EyouJoints arm[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    }
    
    std::cout<< "arm["<<1<<"] can channel: "<<robot_config.motors[1].can_channel<<std::endl;
    arm[1].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[1].pdo_auto_report_ms);
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    arm[0].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[0].pdo_auto_report_ms);
             
    double start_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double target_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double act_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double act_vel[NUM_ARM][NUM_JOINTS_PER_ARM];
    double act_cur[NUM_ARM][NUM_JOINTS_PER_ARM];
    double pos[NUM_ARM][NUM_JOINTS_PER_ARM], vel[NUM_ARM][NUM_JOINTS_PER_ARM], acc[NUM_ARM][NUM_JOINTS_PER_ARM];
    ////////////////////////////////////////Trajectory coefficient generation
    double t = 0.01;
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    while (1)
    {
        for (int i = 0;i < NUM_ARM;++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                act_pos[i][j] = arm[i].get_position(j);
                act_vel[i][j] = arm[i].get_velocity(j);
                act_cur[i][j] = arm[i].get_current(j);
                std::cout << "arm " << i << " | joint " << j << " | act pos: " << act_pos[i][j] << " | act vel: " << act_vel[i][j] << " | act cur: " << act_cur[i][j] << std::endl;
                
            }
        }
       
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}
