#include "serial_sbt_force_sensor.h"
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <signal.h>
#include <fstream>
#include <vector>
#include <termios.h>
#include <fcntl.h>
#include "config_loader.h"
#include "can_fd.h"
#include "can_normal.h"
#include "kinematics.h"
#include "robot_arm.h"
#include "robot_struct.h"


// 假设 num_joints_ 在 kinematics.h 中定义，若未定义请取消下行注释
// constexpr int num_joints_ = 6; 

int main(int argc, char *argv[])
{
    
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robot_config = config_loader.getRobot();
    
    // 初始化 CAN
    can_normal_init(0);
    canfd_init(0,0);
    
    RobotArm single_arm_v1(robot_config);
    single_arm_v1.initialize(100);
    single_arm_v1.set_control_mode(pos_control);
    FILE* fwrite_data = fopen("force_data_load.txt", "w");
    // 初始化串口力传感器
    serial_sbt_force_sensor sensor;
    if (serial_sbt_force_sensor_open(&sensor, "/dev/ttyUSB0", 9600)) {
        printf("cannot open sbt module!!!\n");
        return -1;
    } else {
        printf("open sbt module.\n");
    }

    double zero_read_torque[3] = {0.0};
    double zero_read_current[6] = {0.0};
    

    //std::this_thread::sleep_for(std::chrono::microseconds(10000000));  // 等待传感器稳定


    for (int j = 0; j < 3; ++j) {
        if (robot_config.sensors.enabled[j] == 0)
                continue;
        single_arm_v1.read_torque_sensor(j);
        zero_read_torque[j] = single_arm_v1.get_torque(j);
    }
    
    for (int j = 0; j < 6; ++j) {
        if (robot_config.sensors.enabled[j] == 0)
                continue;
        zero_read_current[j] = single_arm_v1.get_current(j);
    }
    


    int delay_us = 50000;
    double force_data[1000],joint_current[1000][6],joint_torque[1000][3],q_data[1000][6];
    
    for (int i = 0; i < 1000; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (robot_config.sensors.enabled[j] == 0)
                continue;
            single_arm_v1.read_torque_sensor(j);
            single_arm_v1.read_status(j);
            joint_torque[i][j] = single_arm_v1.get_torque(j) - zero_read_torque[j];
            fprintf(fwrite_data, "%.4f ", joint_torque[i][j]);
        }

        for (int j = 0; j < 6; ++j) {
            if (robot_config.motors.enabled[j] == 0)
                continue;
            single_arm_v1.read_status(j);
            joint_current[i][j] = single_arm_v1.get_current(j) - zero_read_current[j];
            fprintf(fwrite_data, "%.4f ", joint_current[i][j]);
        }

        for (int j = 0; j < 6; ++j) {
            if (robot_config.motors.enabled[j] == 0)
                continue;
            single_arm_v1.read_status(j);
            q_data[i][j] = single_arm_v1.get_mdh_position(j);
            fprintf(fwrite_data, "%.4f ",  q_data[i][j]);
        }

        serial_sbt_force_sensor_read(&sensor);
        std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
        std::cout << i << " force: " << sensor.data << std::endl;
        force_data[i] = sensor.data;
        fprintf(fwrite_data, "%.4f\n", force_data[i]);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    
    serial_sbt_force_sensor_close(&sensor);
    
    
    can_normal_close(0);
    fclose(fwrite_data);
    return 0;
}