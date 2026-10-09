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
#include "torque_sensor.h"
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

    if (robot_config.torque_sensors[0].device_type == DEVICE_USBCANII)
    {
        serial_can_open(CH0|CH1);
    }
    EyouJoints arm[NUM_ARM];
    TorSensor torque_sensors[NUM_ARM];
    int pos_arm_index = 0; // 1 for right arm, 0 for left arm
    for (int i = 0; i < NUM_ARM; ++i)
    {
        // new (&torque_sensors[i]) EyoTorSenor(i,NUM_JOINTS_PER_ARM, robot_config.torque_sensors[i].can_channel, 
        //                                     robot_config.torque_sensors[i].ids.data());
        new (&torque_sensors[i]) TorSensor((CanDeviceType)(robot_config.torque_sensors[i].device_type), i, NUM_JOINTS_PER_ARM,
                                            robot_config.torque_sensors[i].ids.data(), robot_config.torque_sensors[i].can_channel);
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    }
    std::cout<< "arm["<<1<<"] can channel: "<<robot_config.motors[pos_arm_index].can_channel<<std::endl;
    arm[pos_arm_index].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[pos_arm_index].pdo_auto_report_ms);
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[1-pos_arm_index].can_channel<<std::endl;
    arm[1-pos_arm_index].init_joints(harmonic_OperateMode_CyclicSyncTorque, robot_config.motors[1-pos_arm_index].pdo_auto_report_ms);
    
    std::ofstream fwrite_data("cur_twin.txt");
    std::vector<double> traj_coef[NUM_JOINTS_PER_ARM];
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double dt = 0.01;
    int T = 1000;
    int data_size = 20000;
    double joint_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double DH_pos[NUM_ARM][NUM_JOINTS_PER_ARM], act_cur[NUM_ARM][NUM_JOINTS_PER_ARM], torque_sensor_read[NUM_ARM][NUM_JOINTS_PER_ARM];
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
            if (!robot_config.motors[1-pos_arm_index].enabled[j]) continue;
            arm[1-pos_arm_index].set_torque(j, 0);
            for (int k = 0; k < 2; k++)
                DH_pos[1-pos_arm_index][j] = arm[1-pos_arm_index].get_mdhpos(j);
        }
        
        for (int j = 0; j < NUM_JOINTS_PER_ARM; j++)
        {
            if (!robot_config.motors[pos_arm_index].enabled[j]) continue;
            DH_pos[pos_arm_index][j] = arm[pos_arm_index].get_mdhpos(j);
            act_cur[pos_arm_index][j] = arm[pos_arm_index].get_current(j);
            torque_sensors[pos_arm_index].read_sensor(j);
            torque_sensor_read[pos_arm_index][j] = torque_sensors[pos_arm_index].get_torque_data(j);
            map_pos[j] = DH_pos[1-pos_arm_index][j] * joint_direction_map[j];
            
            if (std::fabs(map_pos[j]-DH_pos[pos_arm_index][j])>=0.1 && flag[j]==0)
            {
                
                if (count[j] % 500 == 0)
                {
                    // std::cout << "ARM 1| Joint " << j << "|MDH pos: " << DH_pos[pos_arm_index][j] << " | Mapped pos: " << map_pos[j] << std::endl;
                    traj_coef[j] = polynomial_trajectory::calculateCoefficients(DH_pos[pos_arm_index][j], 0.0, 0.0, map_pos[j], 0.0, 0.0, 3.0);
                    t = 0.0;
                }
                
                std::tie(pos[j], vel[j], acc[j]) = polynomial_trajectory::evaluatePolynomial(traj_coef[j], t);
                arm[pos_arm_index].set_mdhpos(j, pos[j]);
                count[j]++;
            }else
            {
                flag[j] = 1;
                count[j] = 0;
                arm[pos_arm_index].set_mdhpos(j, map_pos[j]);

            }
            fwrite_data << DH_pos[pos_arm_index][j]<<" "<<act_cur[pos_arm_index][j]<<" " << torque_sensor_read[pos_arm_index][j] << " ";
            // // 记录右臂的实际位置
            if ((int)(t / dt) % 10 == 0)
                std::cout<< "ARM 1| Joint " << j <<"|MDH pos: "<<DH_pos[pos_arm_index][j] << " torque_sensor_read: "<< torque_sensor_read[pos_arm_index][j] <<std::endl;
            t += dt;
        }
        fwrite_data << std::endl;
        std::this_thread::sleep_until(next_time);
    }
    arm[1-pos_arm_index].all_motor_stop();
    arm[pos_arm_index].all_motor_stop();
    fwrite_data.close();
    serial_can_close(CH0 | CH1);
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}