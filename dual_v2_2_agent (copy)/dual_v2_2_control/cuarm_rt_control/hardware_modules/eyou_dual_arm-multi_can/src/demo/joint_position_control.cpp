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
#include "kinematics.h"
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
    // To store the processed data
    std::ofstream fwrite_data("pos_control.txt");
    double start_pos[NUM_ARM][NUM_JOINTS_PER_ARM], target_pos[NUM_ARM][NUM_JOINTS_PER_ARM], zero_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double act_pos[NUM_ARM][NUM_JOINTS_PER_ARM], act_vel[NUM_ARM][NUM_JOINTS_PER_ARM], act_cur[NUM_ARM][NUM_JOINTS_PER_ARM];
    double pos[NUM_ARM][NUM_JOINTS_PER_ARM], vel[NUM_ARM][NUM_JOINTS_PER_ARM], acc[NUM_ARM][NUM_JOINTS_PER_ARM];
    int stop_flag = 0;
    double t[2],  dt[2];
    //Trajectory coefficient generation
    std::vector<double> traj_coef[NUM_ARM][NUM_JOINTS_PER_ARM];
    for (int i = 0; i < NUM_ARM; i++)
    {
        for (int j = 0;j<NUM_JOINTS_PER_ARM; ++j)
        {
            if (!robot_config.motors[i].enabled[j]) continue;
            start_pos[i][j] = arm[i].get_mdhpos(j);// target_pos is the target degrees in DH coordinate system  
            std::cout << "arm["<<i<<"] joint["<<j<<"] zero pos: after "<<start_pos[i][j]<<std::endl;
            traj_coef[i][j] = polynomial_trajectory::calculateCoefficients(start_pos[i][j], 0.0, 0.0, robot_config.control.position[i].command_degrees[j], 0.0, 0.0, robot_config.control.position[i].T);
        }
        dt[i] = robot_config.control.position[i].dt;
        t[i] = dt[i];
    }  
    
    while(!stop_flag)
    {
        auto clock_start = std::chrono::steady_clock::now();
        stop_flag = true;
        for (int i = 0;i < NUM_ARM;++i)
        {
            if (t[i]<=robot_config.control.position[i].T)
            {
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                {
                    if (!robot_config.motors[i].enabled[j]) continue;
                    std::tie(pos[i][j], vel[i][j], acc[i][j]) = polynomial_trajectory::evaluatePolynomial(traj_coef[i][j], t[i]);
                    arm[i].set_mdhpos(j, pos[i][j]);
                    if ((int)(t[i] / dt[i]) % 10 == 0)
                    {
                        std::cout << "arm " << i << " | joint " << j << " | target pos: " << pos[i][j]
                                << " | act pos: " << act_pos[i][j] << " | act vel: " << act_vel[i][j] << " | act cur: " << act_cur[i][j] << std::endl;
                    }
                }
                t[i]+=dt[i];
                stop_flag = false;
            }
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                act_pos[i][j] = arm[i].get_mdhpos(j);
                act_vel[i][j] = arm[i].get_velocity(j) * robot_config.motors[i].sign[j] / robot_config.motors[i].gear_ratios[j];
                act_cur[i][j] = arm[i].get_current(j);
                fwrite_data << pos[i][j] - start_pos[i][j] << " " << act_pos[i][j]-start_pos[i][j] << " "<< act_vel[i][j]<< " "<< act_cur[i][j]<< " ";     
            }
        }
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
        if (elapsed_us < dt[0] * 1000000)
            std::this_thread::sleep_for(std::chrono::microseconds((long)(dt[0] * 1000000 - elapsed_us)));
        fwrite_data<< elapsed_us << " " ;
        fwrite_data << std::endl;
    }
    // // //////////////////////////////////////Follow Trajectory
    
    fwrite_data.close();
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});

    for (int i = 0; i < NUM_ARM; i++)
    {
        arm[i].hard_stop_all(5);
    }
    return 0;
}
