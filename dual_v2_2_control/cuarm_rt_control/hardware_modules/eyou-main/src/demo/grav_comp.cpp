#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include <math.h>
#include "Matrix.hpp"
#include "kinematics.h"
#include "config_loader.h"
volatile sig_atomic_t exit_requested = 0;
void signal_handler(int signal)
{
    exit_requested = 1;
}

int main()
{
    std::ofstream fwrite_data("grav_comp.txt");
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    if (robot_config.force_control.gravity_compensation.use_tor)
    {
        serial_can_open(CH0|CH1);
        EyoTorSenor dual_arm_sensors(robot_config.hardware.torque_sensor.ids);
    }
    
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    //////////////////////////
    matrix::Matrix<double, NUM_JOINTS/2*4, 1> left_dynamic_param;
    matrix::Matrix<double, NUM_JOINTS/2*4, 1> right_dynamic_param;
    for (int i = 0; i < NUM_JOINTS/2*4; i++)
    {
        left_dynamic_param(i, 0) = robot_config.force_control.gravity_compensation.left_arm_dynamics[i];
        right_dynamic_param(i, 0) = robot_config.force_control.gravity_compensation.right_arm_dynamics[i];
    }
    double act_pos_rad[NUM_JOINTS];
    double left_pos_rad[NUM_JOINTS/2];
    double right_pos_rad[NUM_JOINTS/2];
    matrix::Matrix<double, 7, 28> Y_G;
    matrix::Matrix<double, 7, 1> left_tau_gravity;
    matrix::Matrix<double, 7, 1> right_tau_gravity;
    matrix::Matrix<double, 14, 1> tau_gravity;

    int step = 0;
    int time_interval_ms = robot_config.force_control.dt * 1000; // Convert to milliseconds
    
    while (!exit_requested && step < 1000)
    {
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            act_pos_rad[i] = dual_arm.get_position(i)* DEGREE2RAD;
            // act_vel_rad[i] = dual_arm.get_velocity(i);
            // act_current[i]=dual_arm.get_current(i);
            act_pos_rad[i] = (dual_arm.get_position(i) - robot_config.hardware.motor.init_pos_degrees[i])*DEGREE2RAD 
                            * robot_config.hardware.motor.sign[i] / robot_config.hardware.motor.gear_ratios[i];
            std::cout << "act_pos_angle[" << i << "]" << act_pos_rad[i] << std::endl;
        }
        for (int i = 0; i < 7; i++)
        {
            left_pos_rad[i] = act_pos_rad[i];
            right_pos_rad[i] = act_pos_rad[i + 7];
        }
        kinematics::computeRegression(0, left_pos_rad, Y_G);

        left_tau_gravity = Y_G * left_dynamic_param;

        kinematics::computeRegression(1, right_pos_rad, Y_G);

        right_tau_gravity = Y_G * right_dynamic_param;

        for (int i = 0; i < 7; i++)
        {
            tau_gravity(i, 0) = left_tau_gravity(i, 0);
        }
        for (int i = 7; i < 14; i++)
        {
            tau_gravity(i, 0) = right_tau_gravity(i - 7, 0);
        }
        std::cout << "\n=== Step " << step << " (Time: " << std::fixed << std::setprecision(3) << step * time_interval_ms / 1000.0f << "s) ===" << std::endl;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            std::cout << "tau_gravity[" << i << "]" << float(robot_config.force_control.gravity_compensation.K[i] * tau_gravity(i, 0)) << std::endl;
            if (!robot_config.hardware.motor.enabled[i])continue;
            dual_arm.set_torque(i, float(robot_config.force_control.gravity_compensation.K[i] * tau_gravity(i, 0)));
            // dual_arm.set_torque(i, 0);
            printf("Motor ID: %2d | ", i+1);
            printf("Pos: %5.4f Rad| ", act_pos_rad[i]);
            fwrite_data << act_pos_rad[i] << " ";
        }
        std::this_thread::sleep_for(std::chrono::microseconds(time_interval_ms));
        // printf("\n");
        fwrite_data << std::endl;
    }
    fwrite_data.close();
    dual_arm.canable_close();
}