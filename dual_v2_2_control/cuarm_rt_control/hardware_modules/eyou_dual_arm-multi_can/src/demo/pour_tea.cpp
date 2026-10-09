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
#include "inspire_hand.h"
#include <unistd.h>
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
#include "pos_controller.h"
volatile sig_atomic_t exit_requested = 0;
void signal_handler(int signal)
{
    exit_requested = 1;
}
int main(int argc, char *argv[])
{   
    signal(SIGINT, signal_handler);
    //Load configuration form config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    DemoParamLoader demo_param(base_path + "/demo.yaml");
    PourTea& pour_tea_demo = demo_param.pour_tea;
    RobotConfig& robot_config = config_loader.getRobotConfig();
    std::ofstream fwrite_data("pour_tea.txt");
    // Initialize CAN communication and pdo callback dispatcher
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    std::chrono::milliseconds sleep_time(1000);
    CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    EyouJoints arm[NUM_ARM];
    harmonic_OperateMode mode[2];
    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
        std::cout<< "arm["<<i<<"] can channel: "<<robot_config.motors[i].can_channel<<std::endl;

        if (pour_tea_demo.operation_mode[i] == 1)
            mode[i] = harmonic_OperateMode_CyclicSyncVelocity;
        else
            mode[i] = harmonic_OperateMode_CyclicSyncPosition;
        arm[i].init_joints(mode[i], robot_config.motors[i].pdo_auto_report_ms);
    }
    
    int hand_ids[2] = {robot_config.hands[0].id, robot_config.hands[1].id};
    int hand_channel[2] = {robot_config.motors[0].can_channel, robot_config.motors[1].can_channel};
    InspireHand inspire_hands(hand_ids, hand_channel);

    double zero_pos[NUM_ARM][NUM_JOINTS_PER_ARM], start_pos[NUM_ARM][NUM_JOINTS_PER_ARM],target_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double act_pos[NUM_ARM][NUM_JOINTS_PER_ARM], act_vel[NUM_ARM][NUM_JOINTS_PER_ARM], act_cur[NUM_ARM][NUM_JOINTS_PER_ARM];
    double pos[NUM_ARM][NUM_JOINTS_PER_ARM], vel[NUM_ARM][NUM_JOINTS_PER_ARM], acc[NUM_ARM][NUM_JOINTS_PER_ARM];
    double output[NUM_ARM][NUM_JOINTS_PER_ARM];
    double dt[2], t[2];
    std::vector<double> traj_coef[NUM_ARM][NUM_JOINTS_PER_ARM];
    PController::CosInterpolation joint_controllers[NUM_ARM][NUM_JOINTS_PER_ARM];
    for (int traj_idx = 0; traj_idx < pour_tea_demo.num_points; ++traj_idx)
    {
        if (exit_requested)
            break;
        int all_converge_flag = 0;
        std::cout << "--------------------------------TRAJECTORY: " << traj_idx << " ------------------------" << std::endl;
        ////////////////////////////////////////Trajectory coefficient generation 
        for (int i = 0; i < NUM_ARM; i++)
        {
            for (int j = 0;j<NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                start_pos[i][j] = arm[i].get_mdhpos(j);    
                if (pour_tea_demo.operation_mode[i] == 0)
                {
                    traj_coef[i][j] = polynomial_trajectory::calculateCoefficients(start_pos[i][j], 0.0, 0.0, pour_tea_demo.pour_tea_target[i][traj_idx][j]
                                                                                , 0.0, 0.0, pour_tea_demo.T[i][traj_idx]);
                }else
                {
                    target_pos[i][j] = arm[i].mdh_to_joints(j,pour_tea_demo.pour_tea_target[i][traj_idx][j]);
                    joint_controllers[i][j] = PController::CosInterpolation(
                        robot_config.control.velocity[i].error_tolerance, pour_tea_demo.dt[i][traj_idx], pour_tea_demo.vel_T[i][traj_idx]);
                    joint_controllers[i][j].reset();
                }                                                                     
            }
            dt[i] = pour_tea_demo.dt[i][traj_idx];
            t[i] = dt[i];
        }
        int stop_flag = 0;
        
        //////////////////////////////////////Follow Trajectory
        while(!stop_flag && !exit_requested)
        {
            auto clock_start = std::chrono::steady_clock::now();
            stop_flag = 1;
            for (int i = 0;i < NUM_ARM;++i)
            {
                if (pour_tea_demo.operation_mode[i] == 0)
                {
                    if (t[i]<=pour_tea_demo.T[i][traj_idx])
                    {
                        for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                        {
                            if (!robot_config.motors[i].enabled[j]) continue;
                            std::tie(pos[i][j], vel[i][j], acc[i][j]) = polynomial_trajectory::evaluatePolynomial(traj_coef[i][j], t[i]);
                            if (arm[i].set_mdhpos(j, pos[i][j]) == -1)
                            {
                                exit_requested = true;
                            };
                            if ((int)(t[i] / dt[i]) % 10 == 0)
                            {
                                std::cout << "arm " << i << " | joint " << j << " | target pos: " << pos[i][j]
                                        << " | act pos: " << act_pos[i][j] << " | act vel: " << act_vel[i][j] << " | act cur: " << act_cur[i][j] << std::endl;
                            }
                        }
                        t[i]+=dt[i];
                        stop_flag = false;
                    }
                }else{
                    if (t[i]<=pour_tea_demo.vel_T[i][traj_idx])
                    {
                        for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                        {
                            if (!robot_config.motors[i].enabled[j]) continue;
                            act_pos[i][j] = arm[i].get_position(j);
                            output[i][j] = joint_controllers[i][j].update(target_pos[i][j], act_pos[i][j],
                                                                    robot_config.control.velocity[i].kp[j],robot_config.control.velocity[i].max_velocity[j]);
                            if(arm[i].set_velocity(j, output[i][j]) == -1)
                            {
                                exit_requested = true;
                            };
                        }
                        t[i]+=dt[i];
                        stop_flag = false;
                    }
                }
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                {
                    if (!robot_config.motors[i].enabled[j]) continue;
                    act_pos[i][j] = arm[i].get_mdhpos(j);
                    act_vel[i][j] = arm[i].get_velocity(j);
                    act_cur[i][j] = arm[i].get_current(j);
                    fwrite_data << pos[i][j] << " " << act_pos[i][j]<< " "<< act_vel[i][j]<< " "
                                << act_cur[i][j]<< " " << joint_controllers[i][j].getCurrentError() << " "
                                << output[i][j]<< " "<<joint_controllers[i][j].current_t << " ";    
                }
            }
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
        if (elapsed_us < dt[0] * 1000000)
            std::this_thread::sleep_for(std::chrono::microseconds((long)(dt[0] * 1000000 - elapsed_us)));
        fwrite_data<< elapsed_us << " " ;
        fwrite_data << std::endl;
    }
        std::cout <<"--------------------------------TRAJECTORY:"<< traj_idx <<" Finish------------------------\n"<< std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (pour_tea_demo.use_hand)
        {
            for (int i = 0;i < NUM_ARM;++i)
            {
                inspire_hands.setAngles(i, pour_tea_demo.hand_angles[i][traj_idx]);
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    fwrite_data.close();
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    //canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}