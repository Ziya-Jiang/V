#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "config_loader.h"
#include <math.h>
#include "usb_to_can.h"
#include "callback_dispatcher.h"
////////////////xiyue
bool exit_requested = false;

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
    // canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    // CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    // EyouJoints arm[NUM_ARM];
    double zero_pos[NUM_ARM][NUM_JOINTS_PER_ARM];

    int calibration_arm_index = 1; // 1 for right arm, 0 for left arm

    // for (int i = 0; i < NUM_ARM; ++i)
    // {
    //     new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
    //                                 robot_config.motors[i].ids.data(),
    //                                 robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
    //                                 robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    // }
    
    int cnt = 0;
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double dt = 0.01;
    int T = 1000;
    int data_size = 20000;
    double joint_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double DH_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double DH_pos_rad[NUM_ARM][NUM_JOINTS_PER_ARM];
    double ee_pos_ori[NUM_ARM][6];
    double t = dt;
    matrix::Matrix<double, 4UL, 4UL> trans;



    float trajectory[][7] = {
        {2.454380, -1.162135, -1.739255, 1.519948, -1.451688, 1.342237, -0.319572},
    };
    int num_points = 1;

    while (cnt < num_points)
    {

        for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
        {
            DH_pos_rad[calibration_arm_index][j] = trajectory[cnt][j];
        }
        for (int j = 0; j < 7; ++j) {
            std::cout << DH_pos_rad[calibration_arm_index][j];
            if (j < 6) std::cout << ", ";
        }
        std::cout << "calibration_arm_index: " << calibration_arm_index << std::endl;
        kinematics::forwardKinematics(calibration_arm_index, DH_pos_rad[calibration_arm_index], ee_pos_ori[calibration_arm_index]);
        robotics_math::Eula2Mat(trans, ee_pos_ori[calibration_arm_index]);
        // std::cout << "DH_pos_rad: ";
        // for (int j = 0;j<NUM_JOINTS_PER_ARM;++j){
        //     std::cout <<  DH_pos_rad[calibration_arm_index][j] << ", ";
        // }
        // std::cout << std::endl;

        std::cout << "ee_pos_ori: ";
        for (int j = 0;j<6;++j){
            std::cout << ee_pos_ori[calibration_arm_index][j] << ", ";
        }
        std::cout << std::endl;

        
        std::cout << trans(0, 0) << ", " << trans(0, 1) << ", " << trans(0, 2) << ", " << trans(0, 3) << ", "
                << trans(1, 0) << ", " << trans(1, 1) << ", " << trans(1, 2) << ", " << trans(1, 3) << ", "
                << trans(2, 0) << ", " << trans(2, 1) << ", " << trans(2, 2) << ", " << trans(2, 3) << "\n";
        cnt += 1;
    }
    // arm[0].all_motor_stop();
    // arm[1].all_motor_stop();
    // canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}