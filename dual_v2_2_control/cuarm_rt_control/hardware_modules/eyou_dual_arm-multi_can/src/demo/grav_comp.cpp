#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include <math.h>
#include "Matrix.hpp"
#include "kinematics.h"
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
bool exit_requested = false;
void signal_handler(int signal)
{
    exit_requested = 1;
}

int main()
{
    signal(SIGINT, signal_handler);
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
    arm[1].init_joints(harmonic_OperateMode_CyclicSyncTorque, robot_config.motors[1].pdo_auto_report_ms);
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    arm[0].init_joints(harmonic_OperateMode_CyclicSyncTorque, robot_config.motors[0].pdo_auto_report_ms);
    // To store the processed data
    std::ofstream fwrite_data("pos_control.txt");
    //////////////////////////
    matrix::Matrix<double, NUM_JOINTS_PER_ARM*4, 1> left_dynamic_param;
    matrix::Matrix<double, NUM_JOINTS_PER_ARM*4, 1> right_dynamic_param;
    for (int i = 0; i < NUM_JOINTS_PER_ARM*4; i++)
    {
        left_dynamic_param(i, 0) = robot_config.dynamic[0].dynamic_parameters[i];
        right_dynamic_param(i, 0) = robot_config.dynamic[1].dynamic_parameters[i];
    }
    double mdh_pos_rad[NUM_ARM][NUM_JOINTS_PER_ARM];
    double joint_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double joint_vel[NUM_ARM][NUM_JOINTS_PER_ARM];
    double gravity_comp[NUM_ARM][NUM_JOINTS_PER_ARM];
    matrix::Matrix<double, 7, 28> Y_G;
    matrix::Matrix<double, 14, 1> tau_gravity;
    int stop_flag = 0;
    double dt[2] = {robot_config.control.force.gravity_compensation[0].dt, robot_config.control.force.gravity_compensation[1].dt};
    double t[2] = {0.0};
    while(!stop_flag &&!exit_requested)
    {
        auto clock_start = std::chrono::steady_clock::now();
        stop_flag = true;
        for (int i = 0;i < NUM_ARM;++i)
        {
            if (t[i]<=robot_config.control.force.gravity_compensation[i].T)
            {
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                {
                    if (!robot_config.motors[i].enabled[j]) continue;
                    mdh_pos_rad[i][j] = arm[i].get_mdhpos(j)*DEGREE2RAD;
                    joint_vel[i][j] = arm[i].get_velocity(j);

                }
                kinematics::computeRegression(i, mdh_pos_rad[i], Y_G); 
                matrix::Matrix<double, 7, 1> temp = Y_G* (i == 0 ? left_dynamic_param : right_dynamic_param);
                for (int k = 0; k < NUM_JOINTS_PER_ARM; k++)
                {
                    tau_gravity(k + i * NUM_JOINTS_PER_ARM, 0) = temp(k, 0);
                }
                for (int j = 0; j < NUM_JOINTS_PER_ARM; j++)
                {
                    if (!robot_config.motors[i].enabled[j]) continue;
                    gravity_comp[i][j] =(double)(robot_config.control.force.gravity_compensation[i].stiffness[j] 
                                                * tau_gravity(j + i * NUM_JOINTS_PER_ARM, 0));
                    arm[i].set_torque(j, gravity_comp[i][j]);
                    fwrite_data << gravity_comp[i][j]<< " "<< mdh_pos_rad[i][j]<< " "<< joint_vel[i][j] << " ";
                }
                t[i]+=dt[i];
                stop_flag = false;
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


    fwrite_data.close();
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}