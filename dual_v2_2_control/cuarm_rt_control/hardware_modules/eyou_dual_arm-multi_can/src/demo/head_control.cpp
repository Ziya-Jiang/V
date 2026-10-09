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
#include "usb_to_can.h"
#include "callback_dispatcher.h"
int exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}

int main(int argc, char *argv[])
{
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();
    // Initialize CAN communication and pdo callback dispatcher
    canbox_init({robot_config.motors[2].can_channel});
    CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    EyouJoints head_motors;
    new (&head_motors) EyouJoints(0, 2, robot_config.motors[2].can_channel, robot_config.motors[2].use_pdo,
                                robot_config.motors[2].ids.data(), robot_config.motors[2].enabled.data(),
                                robot_config.motors[2].gear_ratios.data(), robot_config.motors[2].sign.data(),
                                robot_config.kinematics[2].initial_position.data());

    

    head_motors.init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[2].pdo_auto_report_ms);

    std::ofstream fwrite_data("head_traj.txt");
    double start_pos[NUM_JOINTS_HEAD];
    double target_pos[NUM_JOINTS_HEAD];
    double act_pos[NUM_JOINTS_HEAD];
    double act_vel[NUM_JOINTS_HEAD];
    double act_cur[NUM_JOINTS_HEAD];
    std::vector<double> traj_coef[NUM_JOINTS_PER_ARM];
    
    // To store the processed data
    for (int j = 0;j<NUM_JOINTS_HEAD; ++j)
    {
        if (!robot_config.motors[2].enabled[j]) continue;
        start_pos[j] = head_motors.get_mdhpos(j);
        target_pos[j] = robot_config.control.head_position.command_degrees[j];
        traj_coef[j] = polynomial_trajectory::calculateCoefficients(start_pos[j], 0.0, 0.0, target_pos[j], 0.0, 0.0, robot_config.control.head_position.T);
        std::cout << "T: " << robot_config.control.head_position.T << "\n";
        std::cout << "start: " << start_pos[j] <<", target: " << target_pos[j]<<"\n";
    }
    double dt = robot_config.control.head_position.dt;
    double t = dt;
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    while (t <= robot_config.control.head_position.T)
    {
        auto clock_start = std::chrono::steady_clock::now();
        for (int j = 0;j<NUM_JOINTS_HEAD;++j)
        {
            double pos, vel, acc;
            if (!robot_config.motors[2].enabled[j]) continue;
            std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[j], t);
            head_motors.set_mdhpos(j, pos);
            act_pos[j] = head_motors.get_mdhpos(j);
            act_vel[j] = head_motors.get_velocity(j);
            std::cout <<"Joint "<< j <<": " << std::fixed << std::setprecision(3)
                      << " target pos: " << pos << ", act pos: " << act_pos[j] << "\n";
            fwrite_data << pos << " " << act_pos[j] << " "<< act_vel[j]<< " ";
        }
        fwrite_data << std::endl;
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
        if (elapsed_us < dt * 1000000)
            std::this_thread::sleep_for(std::chrono::microseconds((long)(dt * 1000000 - elapsed_us)));
        t += dt;
    }
    fwrite_data.close();
    canbox_close({robot_config.motors[2].can_channel});
    return 0;
}
