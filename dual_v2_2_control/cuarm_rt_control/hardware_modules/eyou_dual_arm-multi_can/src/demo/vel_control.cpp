#include <iostream>
#include <thread>
#include <signal.h>
#include <fstream>
#include "eyou_joints.h"
#include "config_loader.h"
#include "pos_controller.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
volatile sig_atomic_t exit_requested = 0;
void signal_handler(int signal)
{
    exit_requested = 1;
}


int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader config_loader(config_path);
    RobotConfig& robot_config = config_loader.getRobotConfig();
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    CallbackDispatcher::init();
    
    EyouJoints arm[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    }
    
    std::cout<< "arm["<<1<<"] can channel: "<<robot_config.motors[1].can_channel<<std::endl;
    arm[1].init_joints(harmonic_OperateMode_CyclicSyncVelocity, robot_config.motors[1].pdo_auto_report_ms);
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    arm[0].init_joints(harmonic_OperateMode_CyclicSyncVelocity, robot_config.motors[0].pdo_auto_report_ms);
    std::ofstream fwrite_data("vel_control.txt");
    PController::CosInterpolation joint_controllers[NUM_ARM][NUM_JOINTS_PER_ARM];
    ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    
    double target_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double act_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double act_vel[NUM_ARM][NUM_JOINTS_PER_ARM];
    double output[NUM_ARM][NUM_JOINTS_PER_ARM];
    int all_converge_flag = 0;
    ////////////////////////////////////////Trajectory coefficient generation
    for (int i = 0; i < NUM_ARM; i++)
    {
        for (int j = 0;j<NUM_JOINTS_PER_ARM; ++j)
        {
            target_pos[i][j] = arm[i].mdh_to_joints(j, robot_config.control.velocity[i].command_degrees[j]);
            joint_controllers[i][j] = PController::CosInterpolation(
                robot_config.control.velocity[i].error_tolerance, robot_config.control.velocity[i].interpolation_dt, robot_config.control.velocity[i].interpolation_T);
        }
    }     
    //////////////////////////////////////Follow Trajectory
    double dt = robot_config.control.velocity[0].interpolation_dt;
    double t = dt;
    while (!exit_requested && !all_converge_flag)
    {
        if ((int)(t / dt) % 10 == 0)
            std::cout<<"============"<< "t: " << std::fixed << std::setprecision(3) << t << "s" <<"============"<< std::endl<< std::endl;
        all_converge_flag = 1;  
        auto clock_now = std::chrono::steady_clock::now();
        for (int i = 0;i<NUM_ARM;++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                act_pos[i][j] = arm[i].get_position(j);
                output[i][j] = joint_controllers[i][j].update(target_pos[i][j], act_pos[i][j],
                                                robot_config.control.velocity[i].kp[j], robot_config.control.velocity[i].max_velocity[j]);
                arm[i].set_velocity(j, output[i][j]);
                act_vel[i][j] = arm[i].get_velocity(j);
                if (act_vel[i][j]> 10000)
                {
                    exit_requested = 1;
                    std::cout << "Velocity too high, exit requested!" << std::endl;
                }
                act_pos[i][j] = arm[i].joints_to_mdh(j, act_pos[i][j]);
                act_vel[i][j] /= robot_config.motors[i].gear_ratios[j] * robot_config.motors[i].sign[j];
                double error = joint_controllers[i][j].getCurrentError();
                fwrite_data << act_pos[i][j] << " " << output[i][j] << " " << act_vel[i][j] << " " << error << " ";                      
                if ((int)(t / dt) % 10 == 0)
                {
                    std::cout << "Joints ID: " << std::setw(2) << i + 1 << " | "
                            << "Current error: " << std::setw(2) << std::fixed << std::setprecision(3) << joint_controllers[i][j].getCurrentError() << " degrees | "
                            << "Act vel: " << std::setw(7) << std::fixed << std::setprecision(3) << act_vel[i][j] << " dps | "
                            << "Act pos: " << std::setw(5) << std::fixed << std::setprecision(4) << act_pos[i][j] << " degrees" << std::endl;
                }
            }
        }
        
        
        for (int i = 0;i<NUM_ARM;++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                if (!joint_controllers[i][j].isConverged())
                {
                    all_converge_flag = 0;
                    break;
                }else{
                    std::cout << "arm " << i << " | joint " << j << " converged!" << std::endl;
                }                   
            }
        };
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = clock_end - clock_now;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
        if (elapsed_us < dt * 1000000)
        {
            std::this_thread::sleep_for(std::chrono::microseconds((long)(dt * 1000000 - elapsed_us)));
        }
        fwrite_data<< elapsed_us << " " ;
        fwrite_data << std::endl;
        t += dt;
    }
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}
    
