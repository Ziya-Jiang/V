#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "config_loader.h"
//////////////////////////////////////////////////////////sensor
int main()
{
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    EyoTorSenor dual_arm_sensors(robot_config.hardware.torque_sensor.ids);

    dual_arm.canable_init();
    serial_can_open(CH0|CH1);
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    std::ofstream fwrite_data("torque_sensor.txt");
    //---------------------------------------过程数据存储-----------------------------------------------------------
    int step = 0;
    int data_size = 20000;
    int time_interval_us = robot_config.force_control.dt*1000000;
    //------力传感器
    double torque_sensor_read[NUM_JOINTS] = {0.0};
    double torque_sensor_zeropoint[NUM_JOINTS] = {0.0};
    double current_zero[NUM_JOINTS] = {0.0};
    double  joint_cur[NUM_JOINTS] = {0.0};
    for (int j = 0; j < 5; j++)
    {
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            if (!robot_config.hardware.torque_sensor.enabled[i]) continue;
            dual_arm_sensors.read_sensor(i);
            torque_sensor_zeropoint[i] = dual_arm_sensors.get_torque_data(i);
            current_zero[i] = dual_arm.get_current(i);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(time_interval_us));
    }
    
    
    while (step<data_size)
    {
        step++;
        std::cout << "\n=== Step " << step << " (Time: " 
              << std::fixed << std::setprecision(3) 
              << step * time_interval_us / 1000000.0f << "s) ===" << std::endl;
        for (int i = 0;i<NUM_JOINTS;++i)
        {
            if (!robot_config.hardware.torque_sensor.enabled[i]) continue;
            dual_arm_sensors.read_sensor(i);
            torque_sensor_read[i] = dual_arm_sensors.get_torque_data(i) - torque_sensor_zeropoint[i];
            joint_cur[i]= (dual_arm.get_current(i) - current_zero[i]);

            printf("Joint ID: 0x%2x | ", i+1);
            printf("Torque Sensor: %7.3f Nm | Current: %7.3f Nm \n", 
                       torque_sensor_read[i], joint_cur[i]);
            fwrite_data << torque_sensor_read[i] << " " << joint_cur[i] << " ";
        }
        fwrite_data << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(time_interval_us));
    }
    fwrite_data.close();
    dual_arm.canable_close();
    serial_can_close(0);
    return 0;
}