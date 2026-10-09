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
#include "inspire_hand.h"
#include <unistd.h>
#include "config_loader.h"

int main(int argc, char *argv[])
{   
    ///////////////////////////////////////////////////////// Dual arm
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    std::ofstream fwrite_data("pour_tea.txt");
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    InspireHand inspire_hands(robot_config.hardware.hand.ids);
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    int NUM_POINTS = 8;

    double mdh_target_degrees[NUM_POINTS][NUM_JOINTS] = {
        {55.070700, 56.936422, -25.906014, 47.423157, 30.365563, 34.855786, -34.930951,                   -55.070700, 56.936422, 25.906014, 47.423157, -30.365563, 34.855786, 34.930951},//zero_points
        {60.465618, 76.196712, -63.333198, 34.000811, 80.586833, 45.054354, -60.612617,                   -60.465618, 76.196712, 63.333198, 34.000811, -80.586833, 45.054354, 60.612617}, 
        {59.308287, 85.686759, -63.363111, 39.591162, 73.386225, 39.693792, -40.508684,                  -59.308287, 85.686759, 63.363111, 39.591162, -73.386225, 39.693792, 40.508684}, 
        {15.717565, 57.297654, -33.519643, 62.246803, 11.606449, 60.783882, -10.779647,                     -33.738853, 75.439110, -2.483724, 66.237611, -12.083919, 5.491474, 52.380326},
        {9.196185, 49.445976, -29.727350, 75.634341, 25.199300, 41.677312, 30.698880,                    -33.738853, 75.439110, -2.483724, 66.237611, -12.083919, 5.491474, 52.380326}, 
        {35.353115, 49.412745, -25.976990, 71.590505, 11.352458, 42.307883,-14.057323,                  -35.353115, 49.412745, 25.976990, 71.590505, -11.352458, 42.307883, 14.057323}, 
        {59.308287, 85.686759, -63.363111, 39.591162, 73.386225, 39.693792, -40.508684,                   -59.308287, 85.686759, 63.363111, 39.591162, -73.386225, 39.693792, 40.508684}, 
        {55.070700, 56.936422, -25.906014, 47.423157, 29.365563, 34.855786, -34.930951,                 -55.070700, 56.936422, 25.906014, 47.423157, -29.365563, 34.855786, 34.930951} //back
    };
    
    int16_t hand_open_angle[2][6] ={{1000, 1000, 1000, 1000, 1000, 0}, {1000, 1000, 1000, 1000, 1000, 0}};
    int16_t hand_grab_angle[2][6] = {{800, 820, 820, 800, 800, 0}, {800, 820, 820, 800, 800, 0}};
    double start_pos[NUM_JOINTS] = {0.};
    double target_pos[NUM_JOINTS] = {0.};
    double act_pos[NUM_JOINTS] = {0.};
    std::vector<std::vector<double>> traj_coef(NUM_JOINTS);
    for (int traj_idx = 0; traj_idx < NUM_POINTS; ++traj_idx)
    {
        auto start = std::chrono::steady_clock::now();
        auto next_time = start;
        std::cout << "--------------------------------TRAJECTORY: " << traj_idx << " ------------------------" << std::endl;
        if (traj_idx == 1)
        {
            inspire_hands.setAngles(0, hand_open_angle[0]);
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            inspire_hands.setAngles(1, hand_open_angle[1]);
        }
        ////////////////////////////////////////Trajectory coefficient generation
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            start_pos[i] = dual_arm.get_position(i); // start actual pulse of joints
            std::cout << "Joint " << i + 1 << " start_dh_position "
                  << (robot_config.hardware.motor.sign[i] *
                      (start_pos[i] - robot_config.hardware.motor.init_pos_degrees[i])) /
                         robot_config.hardware.motor.gear_ratios[i]
                  << std::endl;
            target_pos[i] = (robot_config.hardware.motor.init_pos_degrees[i] + robot_config.hardware.motor.sign[i] 
                            * mdh_target_degrees[traj_idx][i] * robot_config.hardware.motor.gear_ratios[i]);
            traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos[i], 0.0, 0.0, target_pos[i], 0.0, 0.0, robot_config.position_control.T);
        }
        //////////////////////////////////////Follow Trajectory
        int step = 0;
        double t = robot_config.position_control.dt;
        while (t <= robot_config.position_control.T)
        {
            step++;
            next_time += std::chrono::milliseconds((int64_t)(robot_config.position_control.dt * 1000));
            if (step % 10 == 0)
                std::cout << "\n=== Step " << step << " (t: " << std::fixed << std::setprecision(3) << t << "ms) ===" << std::endl;
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                double pos, vel, acc;
                if (!robot_config.hardware.motor.enabled[i])
                    continue;
                std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
                dual_arm.set_pos(i, pos);
                act_pos[i] = dual_arm.get_position(i);
                if (step % 10 == 0)
                {
                    std::cout << "Joints ID: " << std::setw(2) << i + 1 << " | "
                          << "Ref pos: " << std::setw(7) << std::fixed << std::setprecision(3) << pos << " degrees | "
                          << "Act pos: " << std::setw(5) << std::fixed << std::setprecision(4) << act_pos[i] << " degrees" << std::endl;
                }
                // fwrite_data << pos << ", " << act_pos[i] << " ";
                fwrite_data << pos << " " << act_pos[i] << " " << vel << " " << acc << " ";
            }
            fwrite_data << std::endl;
            std::this_thread::sleep_until(next_time);
            t += robot_config.position_control.dt;
        }
        printf("--------------------------------TRAJECTORY: %d Finish------------------------\n", traj_idx);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (traj_idx == 2)
        {
            inspire_hands.setAngles(0, hand_grab_angle[0]);
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            inspire_hands.setAngles(1, hand_grab_angle[1]);
        }
        if (traj_idx == 6)
        {
            inspire_hands.setAngles(0, hand_open_angle[0]);
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            inspire_hands.setAngles(1, hand_open_angle[1]);
        }
    }
    fwrite_data.close();
    dual_arm.canable_close();
    return 0;
}