#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include <math.h>
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
////////////////xiyue
bool exit_requested = false;
#define CLAMP(x, low_lim, high_lim) ((x) < (low_lim) ? low_lim : ((x) > (high_lim) ? (high_lim) : (x)))

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
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel, robot_config.torque_sensors[0].can_channel});
    CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    EyouJoints arm[NUM_ARM];
    EyoTorSenor torque_sensors[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&torque_sensors[i]) EyoTorSenor(i, NUM_JOINTS_PER_ARM, robot_config.torque_sensors[i].can_channel, 
                                            robot_config.torque_sensors[i].ids.data());
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    }
    std::cout<< "arm["<<1<<"] can channel: "<<robot_config.motors[1].can_channel<<std::endl;
    arm[1].init_joints(harmonic_OperateMode_CyclicSyncVelocity, robot_config.motors[1].pdo_auto_report_ms);
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    arm[0].init_joints(harmonic_OperateMode_CyclicSyncVelocity, robot_config.motors[0].pdo_auto_report_ms);
    /////////////////////////
    
    std::ofstream fwrite_data("admittance.txt");
    
    //---------------------------------------过程数据存储-----------------------------------------------------------
    //------torque senor data
    float torque_sensor_read[NUM_ARM][NUM_JOINTS_PER_ARM];
    float torque_sensor_zeropoint[NUM_ARM][NUM_JOINTS_PER_ARM];
    //------joint processed data
    float admittance_output[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0}, gravity_tor[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    float joint_tor[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    float joint_vel[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    float mdh_pos[NUM_ARM][NUM_JOINTS_PER_ARM] = {0.0};
    //----------------------------------------------main lop-----------------------------------------------------
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double dt = robot_config.control.force.admittance[0].dt; // 期望的控制周期（秒）
    double t = dt;
    while (!exit_requested)
    {
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));
        for (int i = 0; i < NUM_ARM; ++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                mdh_pos[i][j] = arm[i].get_mdhpos(j);
                torque_sensors[i].read_sensor(j);
                torque_sensor_read[i][j] = torque_sensors[i].get_torque_data(j);
                admittance_output[i][j] = torque_sensor_read[i][j] - gravity_tor[i][j];
                admittance_output[i][j] = CLAMP(admittance_output[i][j], -robot_config.control.force.admittance->output_limits[j],
                                                robot_config.control.force.admittance->output_limits[j]);
                arm[i].set_velocity(j, admittance_output[i][j]);
                fwrite_data<< admittance_output[i][j] << " " << torque_sensor_read[i][j]<< " "<< mdh_pos[i][j] << " ";
            }
        }
        std::this_thread::sleep_until(next_time);
        t+= dt;
        fwrite_data << std::endl;
    }   
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    fwrite_data.close();
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel, robot_config.torque_sensors[0].can_channel});
    return 0;
}