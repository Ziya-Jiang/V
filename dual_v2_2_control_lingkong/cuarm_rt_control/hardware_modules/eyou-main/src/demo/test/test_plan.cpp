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
////////////////////////////////////////////////////////////// @brief Define pos control trajectory
int main(int argc, char *argv[])
{
    std::ofstream pos_file("position_data.txt");
    std::ofstream vel_file("velocity_data.txt");
    std::ofstream acc_file("acceleration_data.txt");
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double start_pos[NUM_JOINTS] = {0.};
    double target_pos[NUM_JOINTS] = {0.};
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    std::vector<std::vector<double>> traj_coef(NUM_JOINTS);
    double way_points[16][14] = {{0.857399, 1.18529, -0.172189, 0.859509, 0.539194, 0.396055, -0.962477, -0.830746, 1.18433, 0.096066, 0.858166, -0.490107, 0.381578, 0.99354},
        {0.857399, 1.18529, -0.172189, 0.859509, 0.539194, 0.396055, -0.962477, -0.830746, 1.18433, 0.096066, 0.858166, -0.490107, 0.381578, 0.99354},
        {0.858105, 1.17424, -0.170507, 0.862869, 0.533312, 0.400767, -0.955048, -0.831349, 1.17327, 0.0945561, 0.861559, -0.484747, 0.38647, 0.9861, },
        {0.859496, 1.15244, -0.16719, 0.869497, 0.521708, 0.410061, -0.940391, -0.832538, 1.15147, 0.0915776, 0.868251, -0.474174, 0.396118, 0.971427,} ,
        {0.861445, 1.12189, -0.162539, 0.878784, 0.505443, 0.423086, -0.919845, -0.834203, 1.12091, 0.0874028, 0.877631, -0.459356, 0.409642, 0.950862, },
        {0.863813, 1.08476, -0.156886, 0.890067, 0.485675, 0.438915, -0.894871, -0.836226, 1.08377, 0.0823292, 0.889033, -0.441347, 0.426078, 0.925871, },
        {0.86646, 1.04324, -0.150566, 0.902683, 0.463572, 0.456614, -0.866945, -0.838487, 1.04224, 0.0766565, 0.901783, -0.421212, 0.444457, 0.897928, },
        {0.869247, 0.999534, -0.143913, 0.915966, 0.440303, 0.475246, -0.837547, -0.840869, 0.998528, 0.0706848, 0.915206, -0.400013, 0.463804, 0.868511, },
        {0.872035, 0.955831, -0.137261, 0.929251, 0.417038, 0.493877, -0.808154, -0.843252, 0.954815, 0.0647138, 0.928628, -0.378817, 0.48315, 0.839094, },
        {0.874686, 0.914328, -0.130944, 0.94187, 0.394942, 0.511574, -0.780243, -0.845518, 0.9133, 0.059043, 0.941376, -0.358686, 0.501524, 0.811156, },
        {0.87706, 0.877213, -0.125294, 0.953157, 0.375183, 0.527401, -0.755285, -0.847546, 0.876174, 0.0539718, 0.952778, -0.340685, 0.517957, 0.786172, },
        {0.879013, 0.846676, -0.120646, 0.962444, 0.358926, 0.540423, -0.734752, -0.849216, 0.845628, 0.0497993, 0.962162, -0.325874, 0.53148, 0.765618, },
        {0.880407, 0.824888, -0.11733, 0.96907, 0.347329, 0.549714, -0.720102, -0.850407, 0.823837, 0.0468224, 0.968857, -0.315309, 0.541128, 0.750955, },
        {0.881114, 0.813844, -0.115649, 0.972429, 0.34145, 0.554423, -0.712676, -0.851011, 0.812792, 0.0453134, 0.97225, -0.309953, 0.546019, 0.743523, },
        {0.881114, 0.813844, -0.115649, 0.972429, 0.34145, 0.554423, -0.712676, -0.851011, 0.812792, 0.0453134, 0.97225, -0.309953, 0.546019, 0.743523, },
        {0.881114, 0.813844, -0.115649, 0.972429, 0.34145, 0.554423, -0.712676, -0.851011, 0.812792, 0.0453134, 0.97225, -0.309953, 0.546019, 0.743523, }};
    ////////////////////////////////////////Trajectory coefficient generation
    for (int traj_index = 1;traj_index<16;++traj_index)//我这里没走第一个点，从第一个点往后面走的
    {
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            if (traj_index == 0)
                start_pos[i] = robot_config.hardware.motor.init_pos_degrees[i];//假设电机一开始从零位开始
            else
                start_pos[i] = (robot_config.hardware.motor.init_pos_degrees[i] + robot_config.hardware.motor.sign[i] 
                            * way_points[traj_index-1][i]*RAD2DEGREE * robot_config.hardware.motor.gear_ratios[i]);//假设电机走到位了
            target_pos[i] = (robot_config.hardware.motor.init_pos_degrees[i] + robot_config.hardware.motor.sign[i] 
                            * way_points[traj_index][i]*RAD2DEGREE * robot_config.hardware.motor.gear_ratios[i]);
            traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos[i], 0.0, 0.0, target_pos[i], 0.0, 0.0, 0.3);
        }
        int step = 0;
        double t = 0.02;    
        while (t <= 0.3)
        {
            step++;
            next_time += std::chrono::milliseconds((int64_t)(robot_config.position_control.dt * 1000));
            if (step % 1 == 0)
                std::cout << "\n=== Step " << step << " (t: " << std::fixed << std::setprecision(3) << t << "s) ===" << std::endl;
            double current_pos[NUM_JOINTS] = {0.};
            double current_vel[NUM_JOINTS] = {0.};
            double current_acc[NUM_JOINTS] = {0.};
            
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                std::tie(current_pos[i], current_vel[i], current_acc[i]) = 
                    polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
            }
            for (int i = 0; i < NUM_JOINTS; ++i) {
                pos_file << current_pos[i] << (i < NUM_JOINTS-1 ? " " : "\n");
            }
            
            // 写入速度数据
            for (int i = 0; i < NUM_JOINTS; ++i) {
                vel_file << current_vel[i] << (i < NUM_JOINTS-1 ? " " : "\n");
            }
            
            // 写入加速度数据
            for (int i = 0; i < NUM_JOINTS; ++i) {
                acc_file << current_acc[i] << (i < NUM_JOINTS-1 ? " " : "\n");
            }
            std::this_thread::sleep_until(next_time);
            t += robot_config.position_control.dt;
        }

    }
    //////////////////////////////////////Follow Trajectory
    pos_file.close();
    vel_file.close();
    acc_file.close();
    return 0;
}
