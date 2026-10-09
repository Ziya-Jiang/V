#include <iostream>
#include <thread>
#include <signal.h>
#include <fstream>
#include "eyou_joints.h"
#include "config_loader.h"
#include "pos_controller.h"
volatile sig_atomic_t exit_requested = 0;
void signal_handler(int signal)
{
    exit_requested = 1;
}


int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);
    signal(SIGSEGV, signal_handler);
    double mdh_target_degrees[NUM_JOINTS];
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);

    std::ofstream fwrite_data("vel_control.txt");
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    PController::CosInterpolation joint_controllers[NUM_JOINTS];
    ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncVelocity);

    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double target_pos[NUM_JOINTS] = {0.};
    double act_pos[NUM_JOINTS] = {0.};
    double act_vel[NUM_JOINTS] = {0.};
    double output[NUM_JOINTS] = {0.};
    int all_converge_flag = 0;
    ////////////////////////////////////////Trajectory coefficient generation
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        mdh_target_degrees[i] = robot_config.velocity_control.cmd[i];
        target_pos[i] = (robot_config.hardware.motor.init_pos_degrees[i] + robot_config.hardware.motor.sign[i] 
                        * mdh_target_degrees[i] * robot_config.hardware.motor.gear_ratios[i]);  
        joint_controllers[i] = PController::CosInterpolation(
            robot_config.velocity_control.error_tol, robot_config.velocity_control.dt,robot_config.velocity_control.T);                 
    }               
    //////////////////////////////////////Follow Trajectory
    int step = 0;
    double t = robot_config.velocity_control.dt;

    while (!exit_requested && !all_converge_flag)
    {
        step++;
        next_time += std::chrono::milliseconds((int64_t)(robot_config.position_control.dt * 1000));
        if (step % 10 == 0)
            std::cout << "\n=== Step " << step << " (t: " << std::fixed << std::setprecision(3) << t << "ms) ===" << std::endl;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            act_pos[i] = dual_arm.get_position(i);
            output[i] = joint_controllers[i].update(target_pos[i],act_pos[i],robot_config.velocity_control.Kp[i],robot_config.velocity_control.max_velocity[i]);
            dual_arm.set_velocity(i, output[i]);
            act_vel[i] = dual_arm.get_velocity(i);
            if (step % 10 == 0)
            {
                std::cout << "Joints ID: " << std::setw(2) << i + 1 << " | "
                          << "Current error: " << std::setw(2) << std::fixed << std::setprecision(3) << joint_controllers[i].getTotalError() << " degrees | "
                          << "Act vel: " << std::setw(7) << std::fixed << std::setprecision(3) << act_vel[i] << " dps | "
                          << "Act pos: " << std::setw(5) << std::fixed << std::setprecision(4) << act_pos[i] << " degrees" << std::endl;
            }
            fwrite_data << act_pos[i] << " " << output[i] << " " << act_vel[i] << " ";
            all_converge_flag = 1;
            for (int j = 0; j < NUM_JOINTS; ++j)
            {
                if (!robot_config.hardware.motor.enabled[j])
                    continue;
                if (!joint_controllers[i].isConverged())
                {
                    // std::cout << "Joint " << j + 1 << " not converged, current error: " 
                    //           << joint_controllers[j].getTotalError() << std::endl;
                    all_converge_flag = 0;
                    break;
                }
            }
        }
        fwrite_data << std::endl;
        std::this_thread::sleep_until(next_time);
        t += robot_config.velocity_control.dt;
    }
    dual_arm.all_motor_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    fwrite_data.close();
    dual_arm.canable_close();
    return 0;
}
