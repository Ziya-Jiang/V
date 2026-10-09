#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include <math.h>
#include "config_loader.h"
////////////////xiyue
bool exit_requested = false;
#define CLAMP(x, low_lim, high_lim) ((x) < (low_lim) ? low_lim : ((x) > (high_lim) ? (high_lim) : (x)))
void signal_handler(int signal)
{
    exit_requested = true;
}
double get_current_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}
int main()
{
    signal(SIGINT, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    EyoTorSenor dual_arm_sensors(robot_config.hardware.torque_sensor.ids);
    FILE *fwrite_data = fopen("admittance.txt", "w");
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncVelocity);
    serial_can_open(CH0 | CH1);
    //---------------------------------------过程数据存储-----------------------------------------------------------
    int num_joints = 14;
    int step = 0;
    int data_size = 20000;
    int time_interval_us = 10000;
    //------torque senor data
    float torque_sensor_read[num_joints] = {0.0};
    float torque_sensor_zeropoint[num_joints] = {0.0};
    //------joint processed data
    float admittance_output[num_joints] = {0.0};
    float joint_tor[num_joints] = {0.0};
    float joint_vel[num_joints] = {0.0};
    float joint_pos[num_joints] = {0.0};
    //------gain and damping
    float gain[num_joints] = {-15.00, -15.00, -20.00, -20.0, -100.0, -100.0, -100.0, - 20.00, -20.00, -35.00, -35.0, -100.0, -100.0, -100.0};

    float damping[num_joints] = {0.008, 0.008, 0.008, 0.005, 0.001, 0.001, 0.001,
                                 0.008, 0.008, 0.008, 0.005, 0.001, 0.001, 0.001};
    float inertia[num_joints] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                                 0.5, 0.5, 0.7, 0.5, 0.5, 0.5, 0.5};
    float desired_accel[num_joints] = {0.0};
    float limits[num_joints] = {50., 50., 50., 50., 50., 50., 50.,
                                50., 50., 50., 50., 50., 50., 50.};
    float deadzone[14] = {3.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0,
                          3.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0}; //
    const float VEL_FILTER_ALPHA = 0.2f;                      // low pass filter factor of joints' velocity
    float filtered_joint_vel[num_joints] = {0.0};
    // //---------------------------------------main function-----------------------------------------------------------
    //------------------------------------read the zero-read of torquesensor---------------------------------------------------------
    for (int j = 0; j < 5; ++j)
    {
        for (int i = 0; i < num_joints; i++)
        {
            if (!robot_config.hardware.motor.enabled[i]) continue;
                continue;
            dual_arm_sensors.read_sensor(i);
            torque_sensor_zeropoint[i] = dual_arm_sensors.get_torque_data(i);
        }
        usleep(time_interval_us);
    }
    for (int i = 0; i < num_joints; i++)
        printf("sensor %d data %f\n", i, torque_sensor_zeropoint[i]);
    printf("\n\n");
    int j = 0;
    // fprintf(fwrite_data, "\n\n");
    //----------------------------------------------main loop-----------------------------------------------------
    double prev_time = get_current_time();
    double target_dt = time_interval_us / 1e6; // 期望的控制周期（秒）
    while (!exit_requested && step < data_size)
    {
        double current_time = get_current_time();
        double actual_dt = current_time - prev_time;
        prev_time = current_time;
        step++;
        double remaining_time = target_dt - actual_dt;
        if (remaining_time > 0)
        {
            usleep(remaining_time * 1e6); // 精确补足周期
        }
        std::cout << "\n=== Step " << step << " (Time: " << std::fixed << std::setprecision(3) << step * time_interval_us / 1000000.0f << "s) ===" << std::endl;

        for (int i = 0; i < num_joints; ++i)
        {
            if (!robot_config.hardware.motor.enabled[i]) continue;
            dual_arm_sensors.read_sensor(i);
            torque_sensor_read[i] = dual_arm_sensors.get_torque_data(i) - torque_sensor_zeropoint[i];
            if (fabs(torque_sensor_read[i]) < deadzone[i])
            {
                torque_sensor_read[i] = 0.0;
            }

            joint_vel[i] = dual_arm.get_velocity(i);
            filtered_joint_vel[i] = VEL_FILTER_ALPHA * joint_vel[i] +
                                    (1 - VEL_FILTER_ALPHA) * filtered_joint_vel[i];
            desired_accel[i] = (gain[i] * torque_sensor_read[i] - damping[i] * filtered_joint_vel[i]) / inertia[i];
            // desired_accel[i] = (gain[i]*torque_sensor_read[i] - damping[i]*admittance_output[i])
            //               /inertia[i];
            admittance_output[i] += desired_accel[i] * actual_dt;
            admittance_output[i] = CLAMP(admittance_output[i], -limits[i], limits[i]);
            admittance_output[i] *= 0.95;
            dual_arm.set_velocity(i, admittance_output[i]);
            joint_pos[i] = dual_arm.get_position(i);
            printf("Motor ID: %2d | ", i+1);
            printf("Torque: %7.3f Nm | ",
                        torque_sensor_read[i]);
            printf("Admit: %7.3f rad/s | ", admittance_output[i]);
            printf("Vel: %7.3f rad/s | ", joint_vel[i]);
            printf("Pos: %5.4f\n degrees\n", joint_pos[i]);
            fprintf(fwrite_data, "%f %f %f %f %f %f ", torque_sensor_read[i], admittance_output[i], joint_vel[i], desired_accel[i],  joint_tor[i], actual_dt);
        }
    }
    dual_arm.all_motor_stop();

    fclose(fwrite_data);
    dual_arm.canable_close();
    return 0;
}