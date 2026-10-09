#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "config_loader.h"
#include <math.h>
#include "Matrix.hpp"
#include "kinematics.h"
////////////////xiyue
bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = 1;
}
void xml_input()
{
    // Left Arm
    // double left_a[AXISNUM]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 62.0};
    // double left_alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD};
    // double left_d[AXISNUM]{85.0, 0.0, 250.0, 0.0, 210.0, 0.0, 0.0};
    double left_offset[AXISNUM]{0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, -90 * DEGREE2RAD, 0 * DEGREE2RAD};

    double left_a[AXISNUM]{0.0, -0.155, 0.128, 0.219, -0.153, -0.25, 62.172};
    double left_alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.089 * DEGREE2RAD, 90.103 * DEGREE2RAD, -89.96 * DEGREE2RAD, 90.017 * DEGREE2RAD, -90.125 * DEGREE2RAD, -89.908 * DEGREE2RAD};
    double left_d[AXISNUM]{85.0, -0.16, 250.289, -0.07, 209.883, 0.198, 0.11};
    // double left_offset[AXISNUM]{0 * DEGREE2RAD, 0.329 * DEGREE2RAD, 3.788 * DEGREE2RAD, 4.132 * DEGREE2RAD, -2.9770 * DEGREE2RAD, -90.863 * DEGREE2RAD, -5.622 * DEGREE2RAD};

    double left_sign[AXISNUM]{1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double left_base_to_world[6]{0.0, 143.31, 332.44, -60 * DEGREE2RAD, 0.0, 0.0};
    double left_init_mdhpos[AXISNUM]{
        2326350,
        2963120,
        1123510,
        -1727964,
        138168,
        -1304367,
        -48581,
    };
    for (int i = 0; i < AXISNUM; i++)
    {
        g_robot[0].m_mdh.a[i] = left_a[i];
        g_robot[0].m_mdh.alpha[i] = left_alpha[i];
        g_robot[0].m_mdh.d[i] = left_d[i];
        g_robot[0].m_mdh.offset[i] = left_offset[i];
        g_robot[0].m_mdh.sign[i] = left_sign[i];
        g_robot[0].m_mdh.m_init_mdhpos[i] = left_init_mdhpos[i];
    }
    for (int i = 0; i < 6; i++)
    {
        g_robot[0].m_mdh.base_to_world[i] = left_base_to_world[i];
    }

int main()
{
    xml_input();
    signal(SIGINT, signal_handler);
    signal(SIGSEGV, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    FILE *fwrite_data = fopen("pos.txt", "a"); // save data
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    //---------------------------------------过程数据存储-----------------------------------------------------------
    int step = 0;
    int data_size = 20000;
    double dt = robot_config.force_control.dt;
    double joint_pos[NUM_JOINTS];
    double mdh_pos[NUM_JOINTS];
    double t = dt;
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    //----------------------------------------------main loop-----------------------------------------------------
    while (!exit_requested && t<robot_config.force_control.T)
    {
        if ((int)(t / robot_config.force_control.dt) % 10 == 0)
            std::cout << "\n=== "  << " (Time: " << std::fixed << std::setprecision(3) << t << "s) ===" << std::endl;
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));
        for (int i = 0;i<NUM_JOINTS;++i)
        {
            if (!robot_config.hardware.motor.enabled[i]) continue;
            dual_arm.set_torque(i, 0);
            joint_pos[i] = (robot_config.hardware.motor.sign[i] * 
                            (dual_arm.get_position(i) - robot_config.hardware.motor.init_pos_degrees[i]))
                            /robot_config.hardware.motor.gear_ratios[i];
            std::cout<< "Motor ID "<< i+1 << " | Pos " << joint_pos[i] << "degrees " <<std::endl;
        }
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