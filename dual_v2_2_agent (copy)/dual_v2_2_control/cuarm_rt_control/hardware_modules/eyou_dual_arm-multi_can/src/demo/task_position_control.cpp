#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include "eyou_joints.h"
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
#include "kinematics.h"
#include "cartesian_plan.h"

volatile sig_atomic_t exit_requested = 0;

void signal_handler(int signal) {
    exit_requested = 1;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);
    
    // Initialize configuration and hardware
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();
    EyouJoints arm[NUM_ARM];
    CartesianTrajectoryPlanner arm_planner[NUM_ARM];
    CartesianTrajectory cartesian_traj[NUM_ARM];
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    CallbackDispatcher::init();
    double dt[NUM_ARM], t[NUM_ARM];
    double l7_in_world[NUM_ARM][6];
    double eef_in_world[NUM_ARM][6];
    double act_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double q_in[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    double q_out[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    // Open file for writing data
    std::ofstream fwrite_data("cartesian.txt");
    double test[2][7] = {
                            {55.070700, 56.936422, -25.906014, 47.423157, 30.365563, 34.855786, -38.930951},
                            {-55.070700, 56.936422, 25.906014, 47.423157, -30.365563, 34.855786, 38.930951}};
    // Initialize arms
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arm[i]) EyouJoints(i, NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, 
                               robot_config.motors[i].use_pdo,
                               robot_config.motors[i].ids.data(),
                               robot_config.motors[i].enabled.data(), 
                               robot_config.motors[i].gear_ratios.data(),
                               robot_config.motors[i].sign.data(), 
                               robot_config.kinematics[i].initial_position.data());
       

        //arm[i].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[i].pdo_auto_report_ms);
        for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
        {
            arm[i].zero_mdh_pos[j] = test[i][j];
            q_in[i][j] = arm[i].zero_mdh_pos[j]*DEGREE2RAD;
        }
            
        kinematics::forwardKinematics(i, q_in[i], l7_in_world[i]);
        kinematics::addTool(eef_in_world[i], l7_in_world[i]);
        std::cout<< "Arm " << i << " start eef in world frame: "<<std::endl;
        kinematics::show(eef_in_world[i]);
        arm_planner[i] = CartesianTrajectoryPlanner(i, 0);
        cartesian_traj[i] = {
            {eef_in_world[i][0], eef_in_world[i][1], eef_in_world[i][2], 
            eef_in_world[i][3], eef_in_world[i][4], eef_in_world[i][5]},
            {robot_config.control.position[i].cartesian_command[0], robot_config.control.position[i].cartesian_command[1], robot_config.control.position[i].cartesian_command[2], 
            robot_config.control.position[i].cartesian_command[3], robot_config.control.position[i].cartesian_command[4], robot_config.control.position[i].cartesian_command[5]},
            robot_config.control.position[i].T
        };

        arm_planner[i].addTrajectory(cartesian_traj[i]);
        dt[i] = robot_config.control.position[i].dt;
        t[i] = dt[i];
    }
    int stop_flag = 0;
    // Main control loop
    while (!exit_requested && !stop_flag) {
        auto clock_start = std::chrono::steady_clock::now();
        
        double current_pos[NUM_JOINTS_PER_ARM];
        stop_flag = true;
        for (int i = 0;i<NUM_ARM;++i)
        {
            if (arm_planner[i].isTrajectoryActive())
            {
                if (arm_planner[i].getNextJointAngles(q_out[i], q_in[i], t[i]))
                {
                    for (int j = 0; j < NUM_JOINTS_PER_ARM; j++) {
                        if (!robot_config.motors[i].enabled[j]) continue;
                        // arm[i].set_mdhpos(i, q_out[i][j]*RAD2DEGREE);
                        // act_pos[i][j] = arm[i].get_mdhpos(i);
                        fwrite_data << q_out[i][j]*RAD2DEGREE << " " << act_pos[i][j]*RAD2DEGREE << " ";
                        q_in[i][j] = q_out[i][j];
                    }
                }
                t[i]+=dt[i];
                stop_flag = false;
                
            }
        }
        fwrite_data << std::endl;
        // Timing control
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
        if (elapsed_us < dt[0] * 1000000)
            std::this_thread::sleep_for(std::chrono::microseconds((long)(dt[0] * 1000000 - elapsed_us)));
    }
    
    // Cleanup
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    fwrite_data.close();
    return 0;
}