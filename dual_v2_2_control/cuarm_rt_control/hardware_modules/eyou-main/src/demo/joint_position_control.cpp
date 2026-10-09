#include <type_traits>
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

template<typename T, typename = std::enable_if<std::is_same<T, float>::value || std::is_same<T, double>::value || std::is_same<T, int32_t>::value>>
void fix_motor_initial_value(EyouJoints& dual_arm, int* enable_motors, T* initial_val, int* gear_ratio, int* sign, bool is_pulse, float* read_position=nullptr, hint32* read_pulse=nullptr){
    for (int i = 0; i < NUM_JOINTS; ++i){
        if (!enable_motors[i])
            continue;

        //Read current motor pulse and position
        int ret_val;
        hint32 pulse = dual_arm.get_position_pulse(i, &ret_val); // start actual pulse of joints
        if (ret_val != HARMONIC_SUCCESS){
            std::cerr << "Error reading pulse for joint " << i << std::endl;
            throw std::runtime_error("Failed to read motor " + std::to_string(i) +" pulse");
        }
        float pos = (pulse - initial_val[i]) * PULSE2DEG * sign[i] / gear_ratio[i];

        //Adjust initial value if needed
        float offset = 360.0f * gear_ratio[i] * sign[i];
        if (pos >= 180.0f){
            initial_val[i] += is_pulse ? offset / PULSE2DEG : offset;
        }else if (pos <= -180.0f){
            initial_val[i] -= is_pulse ? offset / PULSE2DEG : offset;
        }
        
        //Update read values
        if (read_position!=nullptr) read_position[i] = pos;
        if (read_pulse!=nullptr) read_pulse[i] = pulse;
        usleep(100);
    }
}

int main(int argc, char *argv[])
{
    double mdh_target_degrees[NUM_JOINTS] = {0.};
    bool valid_input = false;
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    std::ofstream fwrite_data("pos_control.txt");
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double start_pos[NUM_JOINTS] = {0.};
    double target_pos[NUM_JOINTS] = {0.};
    double act_pos[NUM_JOINTS] = {0.};
    double act_cur[NUM_JOINTS] = {0.};
    std::vector<std::vector<double>> traj_coef(NUM_JOINTS);
    fix_motor_initial_value<double>(dual_arm, robot_config.hardware.motor.enabled, robot_config.hardware.motor.init_pos_degrees, robot_config.hardware.motor.gear_ratios, robot_config.hardware.motor.sign, false);
    ////////////////////////////////////////Trajectory coefficient generation
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        mdh_target_degrees[i] = robot_config.position_control.cmd[i];
        if (!robot_config.hardware.motor.enabled[i])
            continue;
        start_pos[i] = dual_arm.get_position(i); // start actual pulse of joints
        std::cout << "Joint " << i + 1 << " start_dh_position "
                  << (robot_config.hardware.motor.sign[i] *
                      (start_pos[i] - robot_config.hardware.motor.init_pos_degrees[i])) /
                         robot_config.hardware.motor.gear_ratios[i]
                  << std::endl;
        target_pos[i] = (robot_config.hardware.motor.init_pos_degrees[i] + robot_config.hardware.motor.sign[i] 
                        * mdh_target_degrees[i] * robot_config.hardware.motor.gear_ratios[i]);
        traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos[i], 0.0, 0.0, target_pos[i], 0.0, 0.0, robot_config.position_control.T);
    }
    //////////////////////////////////////Follow Trajectory
    int step = 0;
    double t = robot_config.position_control.dt;
    while (t <= robot_config.position_control.T)
    {
        step++;
        next_time += std::chrono::milliseconds((int64_t)(robot_config.position_control.dt * 1000));
        // if (step % 1 == 0)
        //     std::cout << "\n=== Step " << step << " (t: " << std::fixed << std::setprecision(3) << t << "s) ===" << std::endl;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            double pos, vel, acc;
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
            dual_arm.set_pos(i, pos);
            act_pos[i] = dual_arm.get_position(i);
            act_cur[i] = dual_arm.get_current(i);
            if (step % 10 == 0)
            {
                std::cout << "Joints ID: " << std::setw(2) << i + 1 << " | "
                          << "Ref pos: " << std::setw(7) << std::fixed << std::setprecision(3) << pos << " degrees | "
                          << "Act pos: " << std::setw(5) << std::fixed << std::setprecision(4) << act_pos[i] << " degrees" << std::endl;
            }
            fwrite_data << pos << " " << act_pos[i] << " "<<act_cur[i]<< " ";
        }
        fwrite_data << std::endl;
        std::this_thread::sleep_until(next_time);
        t += robot_config.position_control.dt;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    fwrite_data.close();
    dual_arm.canable_close();
    return 0;
}