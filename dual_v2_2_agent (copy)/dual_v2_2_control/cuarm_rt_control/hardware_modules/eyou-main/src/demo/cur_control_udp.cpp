#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#include <unistd.h>
#include "eyou_joints.h"
#include "cuarm_udp.h"
#include "cuarm_state.h"
#include "kinematics.h"
#include "config_loader.h"

bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGSEGV, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    FILE *fwrite_data = fopen("pos.txt", "a"); // save data
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    //--------------------------------------------------------------------------------------------------
    udp_node udp;
    udp_init(&udp, (char *)"192.168.3.95", (char *)"192.168.3.42", 10085, 10086);
    DualArmState state;
    state.JointSize = 14;
    state.EEPosSize = 6;
    state.EEOriSize = 6;
    //--------------------------------------------------------------------------------------------------
    double dt = robot_config.force_control.dt, t = dt;
    double joint_pos[NUM_JOINTS]={0.}, mdh_rad[2][7]={0.}, ee_pos_ori[2][6]={0.};
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    int arm_i = 1;
    //----------------------------------------------main loop-----------------------------------------------------
    while (!exit_requested && t<robot_config.force_control.T)
    {
        if ((int)(t / robot_config.force_control.dt) % 10 == 0)
            std::cout << "\n=== "  << " (Time: " << std::fixed << std::setprecision(3) << t << "s) ===" << std::endl;
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));
        //std::cout<<"JOINT SIZE "<<(int) state.JointSize << std::endl;
        for (int i = 0;i<NUM_JOINTS;++i)
        {
            if (!robot_config.hardware.motor.enabled[i]) continue;
            joint_pos[i] = dual_arm.get_position(i);
            // mdh_rad[i / 7][i % 7] = (robot_config.hardware.motor.sign[i] * 
            //                 (joint_pos[i] - robot_config.hardware.motor.init_pos_degrees[i]))*DEG2RAD
            //                 /robot_config.hardware.motor.gear_ratios[i];
            state.JointPos[i] = mdh_rad[i / 7][i % 7];
            std::cout<< "Motor ID "<< i+1 << " | Pos " << joint_pos[i] << " deg " <<std::endl;
            //std::cout<< "Motor ID "<< i << " i/7 " << i / 7 << " i%7: " << i % 7 << " mdh_rad[i / 7][i % 7]: " << mdh_rad[i / 7][i % 7] << std::endl;
            //std::cout<< "Motor ID "<< i+1 << " | Pos " << mdh_rad[i / 7][i % 7] << " rad " <<std::endl;
        }
        kinematics::forwardKinematics(arm_i, mdh_rad[arm_i], ee_pos_ori[arm_i]);
        for(int i = 0; i < 3; i++)
        {
            state.EndEffPos[arm_i*3+i] = ee_pos_ori[arm_i][i];
            state.EndEffOri[arm_i*3+i] = ee_pos_ori[arm_i][i+3];
            //std::cout<< "EEF "<< arm_i << " | Pos " << ee_pos_ori[arm_i][i] << " | Ori " << ee_pos_ori[arm_i][i+3] <<std::endl;
        }
        udp_pack_dual_arm_state(&udp, &state);
        udp_send(&udp);
        std::this_thread::sleep_until(next_time);
        t += dt;
    }
    dual_arm.all_motor_stop();
    dual_arm.canable_close();
    fprintf(fwrite_data, "{");
    for (int i = 0;i<NUM_JOINTS;++i)
    {
        if (!robot_config.hardware.motor.enabled[i]) continue;
        fprintf(fwrite_data, "%f, ", joint_pos[i]);
    }
    fprintf(fwrite_data, "},\n");
    fclose(fwrite_data);
    return 0;
}