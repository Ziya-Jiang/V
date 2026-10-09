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
bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
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
    arm[1].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[1].pdo_auto_report_ms);
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    arm[0].init_joints(harmonic_OperateMode_CyclicSyncTorque, robot_config.motors[0].pdo_auto_report_ms);
    
    std::ofstream fwrite_data("cur_twin.txt");
    std::vector<double> traj_coef[NUM_JOINTS_PER_ARM];
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double dt = 0.01;
    int T = 1000;
    int data_size = 20000;
    double joint_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double DH_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double map_pos[NUM_JOINTS_PER_ARM];
    double pos[NUM_JOINTS_PER_ARM], vel[NUM_JOINTS_PER_ARM], acc[NUM_JOINTS_PER_ARM];
    double t = dt;
    const int joint_direction_map[NUM_JOINTS_PER_ARM] = {-1, 1, -1, 1, -1, 1, -1};
    int flag[NUM_JOINTS_PER_ARM] ={0};
    for (int k = 0; k < 10; k++)
    {
        for (int i = 0; i < NUM_ARM; i++)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; j++)
            {
                DH_pos[i][j] = arm[i].get_mdhpos(j);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    int count[NUM_JOINTS_PER_ARM] = {0};
    while (!exit_requested && t < T)
    {
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));
        if ((int)(t / dt) % 10 == 0)
        {
            std::cout << "======================= Time: " << std::fixed << std::setprecision(2) << t << "s =======================" << std::endl;
        }

        // 先获取左臂的状态和设置零力矩
        for (int j = 0; j < NUM_JOINTS_PER_ARM; j++)
        {
            if (!robot_config.motors[0].enabled[j]) continue;
            arm[0].set_torque(j, 0);
            for (int k = 0; k < 2; k++)
                DH_pos[0][j] = arm[0].get_mdhpos(j);
        }
        // fwrite_data << std::endl;
        

        for (int j = 0; j < NUM_JOINTS_PER_ARM; j++)
        {
            if (!robot_config.motors[1].enabled[j]) continue;
            DH_pos[1][j] = arm[1].get_mdhpos(j);
            map_pos[j] = DH_pos[0][j] * joint_direction_map[j];
            
            if (std::fabs(map_pos[j]-DH_pos[1][j])>=0.15 && flag[j]==0)
            {
                
                if (count[j] % 500 == 0)
                {
                    // std::cout << "ARM 1| Joint " << j << "|MDH pos: " << DH_pos[1][j] << " | Mapped pos: " << map_pos[j] << std::endl;
                    traj_coef[j] = polynomial_trajectory::calculateCoefficients(DH_pos[1][j], 0.0, 0.0, map_pos[j], 0.0, 0.0, 3.0);
                    t = 0.0;
                }
                
                std::tie(pos[j], vel[j], acc[j]) = polynomial_trajectory::evaluatePolynomial(traj_coef[j], t);
                arm[1].set_mdhpos(j, pos[j]);
                count[j]++;
            }else
            {
                flag[j] = 1;
                count[j] = 0;
                arm[1].set_mdhpos(j, map_pos[j]);
            }
            fwrite_data << pos[j]<< " ";
            // // 记录右臂的实际位置
            if ((int)(t / dt) % 10 == 0)
                std::cout<< "ARM 1| Joint " << j <<"|MDH pos "<<DH_pos[1][j] << " | Mapped pos "<< map_pos[j]<<std::endl;
        }
        fwrite_data << std::endl;
        std::this_thread::sleep_until(next_time);
        t += dt;
    }
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    fwrite_data.close();
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}