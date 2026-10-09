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
#include "cuarm_udp.h"
#include "cuarm_state.h"
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
    int pos_arm_index = 1; // 1 for right arm, 0 for left arm

    std::cout<< "arm["<<1<<"] can channel: "<<robot_config.motors[1].can_channel<<std::endl;
    arm[pos_arm_index].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[1].pdo_auto_report_ms);
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    arm[1-pos_arm_index].init_joints(harmonic_OperateMode_CyclicSyncTorque, robot_config.motors[0].pdo_auto_report_ms);
    ///////////////////
    udp_node udp;
    udp_init(&udp, (char *)"192.168.3.42", (char *)"192.168.3.95", 10085, 10086);
    dual_arm_state state;
    state.JointSize = 14;
    state.EEPosSize = 6;
    state.EEOriSize = 6;
    //////////////////////////
    FILE *fwrite_data = fopen("joint_pos_data.txt", "a"); // save data
    FILE *fwrite_data2 = fopen("end_effector_posori.txt", "a"); // save data
    std::vector<double> traj_coef[NUM_JOINTS_PER_ARM];
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double dt = 0.01;
    int T = 1000;
    int data_size = 20000;
    double joint_pos[NUM_ARM][NUM_JOINTS_PER_ARM], zero_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double DH_pos[NUM_ARM][NUM_JOINTS_PER_ARM], DH_pos_rad[NUM_ARM][NUM_JOINTS_PER_ARM], ee_pos_ori[NUM_ARM][6];
    double map_pos[NUM_JOINTS_PER_ARM];
    double pos[NUM_JOINTS_PER_ARM], vel[NUM_JOINTS_PER_ARM], acc[NUM_JOINTS_PER_ARM];
    double t = dt;
    const int joint_direction_map[NUM_JOINTS_PER_ARM] = {-1, 1, -1, 1, -1, 1, -1};
    int flag[NUM_JOINTS_PER_ARM] ={0};

    for (int i = 0;i<NUM_ARM; ++i)
    {
        for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
        {
            if (!robot_config.motors[i].enabled[j]) continue;
            zero_pos[i][j] = arm[i].get_mdhpos(j);
            std::cout << "arm["<<i<<"] joint["<<j<<"] zero pos: "<<zero_pos[i][j]<<std::endl;
            if ((zero_pos[i][j] - arm[i].joints_to_mdh(j, robot_config.kinematics[i].initial_position[j])) >=180.0)
            {
                std::cout << "arm " << i << "motor " << j <<" lack one circle"<< std::endl;
                robot_config.kinematics[i].initial_position[j] = robot_config.kinematics[i].initial_position[j] + 360 * robot_config.motors[i].gear_ratios[j] * robot_config.motors[i].sign[j];
                arm[i].init_mdh_degrees_[j] = robot_config.kinematics[i].initial_position[j];
            }else if ((zero_pos[i][j] - arm[i].joints_to_mdh(j, robot_config.kinematics[i].initial_position[j]))  <=-180.0)
            {
                std::cout << "arm " << i << "motor " << j <<" before "<<  arm[i].init_mdh_degrees_[j] << std::endl;
                robot_config.kinematics[i].initial_position[j] = robot_config.kinematics[i].initial_position[j] - 360 * robot_config.motors[i].gear_ratios[j] * robot_config.motors[i].sign[j];
                arm[i].init_mdh_degrees_[j] = robot_config.kinematics[i].initial_position[j];
                
            }
        }
    }
    
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
            map_pos[j] = DH_pos[1-pos_arm_index][j] * joint_direction_map[j];
            DH_pos_rad[pos_arm_index][j] = DH_pos[pos_arm_index][j]*DEGREE2RAD;
            if (std::fabs(map_pos[j]-DH_pos[pos_arm_index][j])>=0.15 && flag[j]==0)
            {
                
                if (count[j] % 500 == 0)
                {
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
            // // 记录右臂的实际位置
            if ((int)(t / dt) % 10 == 0)
                std::cout<< "ARM 1| Joint " << j <<"|MDH pos "<<DH_pos[pos_arm_index][j] << " | Mapped pos "<< map_pos[j]<<std::endl;
        }
        kinematics::forwardKinematics(pos_arm_index, DH_pos_rad[pos_arm_index], ee_pos_ori[pos_arm_index]);
        for(int i = 0; i < 3; i++)
        {
            state.EndEffPos[pos_arm_index*3+i] = ee_pos_ori[pos_arm_index][i];
            state.EndEffOri[pos_arm_index*3+i] = ee_pos_ori[pos_arm_index][i+3];
            std::cout<< "EEF "<< pos_arm_index << " | Pos " << ee_pos_ori[pos_arm_index][i] << " | Ori " << ee_pos_ori[pos_arm_index][i+3] <<std::endl;
        }
        udp_pack_dual_arm_state(&udp, &state);
        udp_send(&udp);

        std::this_thread::sleep_until(next_time);
        t += dt;
    }
    fprintf(fwrite_data, "{");
    fprintf(fwrite_data2, "{");
    for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
    {
        if (!robot_config.motors[pos_arm_index].enabled[j]) continue;
        fprintf(fwrite_data, "%f, ",  DH_pos[pos_arm_index][j]);
    }
    for(int i = 0; i < 6; i++)
    {
        fprintf(fwrite_data2, "%f, ",  ee_pos_ori[pos_arm_index][i]);
    }
    fprintf(fwrite_data, "},\n");
    fprintf(fwrite_data2, "},\n");
    fclose(fwrite_data2);
    fclose(fwrite_data);
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}