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
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    EyouJoints arm[NUM_ARM];
    double zero_pos[NUM_ARM][NUM_JOINTS_PER_ARM];

    int calibration_arm_index = 1; // 1 for right arm, 0 for left arm

    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    }
    
    std::cout<< "arm["<<calibration_arm_index<<"] can channel: "<<robot_config.motors[calibration_arm_index].can_channel<<std::endl;
    arm[calibration_arm_index].init_joints(harmonic_OperateMode_CyclicSyncTorque, robot_config.motors[calibration_arm_index].pdo_auto_report_ms);
    // std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    // arm[0].init_joints(harmonic_OperateMode_CyclicSyncTorque, robot_config.motors[0].pdo_auto_report_ms);
   

    FILE *fwrite_data = fopen("joint_pos_data.txt", "a"); // save data
    FILE *fwrite_data2 = fopen("end_effector_posori.txt", "a"); // save data
    

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
    while (!exit_requested && t < T)
    {
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));
        for (int i = 0;i<NUM_ARM;++i)
        {
            for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                arm[i].set_torque(j, 0);
                DH_pos[i][j] = arm[i].get_mdhpos(j);
                DH_pos_rad[i][j] = DH_pos[i][j] * DEGREE2RAD;
                if ((int)(t / dt) % 10 == 0)
                    std::cout<< "ARM "<< i << "| Joint " << j <<"|Joint pos "<<joint_pos[i][j] << " | MDH pos "<< DH_pos[i][j]<<std::endl;
            }
        }
        kinematics::forwardKinematics(calibration_arm_index, DH_pos_rad[calibration_arm_index], ee_pos_ori[calibration_arm_index]);
        robotics_math::Eula2Mat(trans, ee_pos_ori[calibration_arm_index]);
        std::cout << "EE pos ori: ";
        for (int i=0; i<3; ++i)
        {   
            for(int j =0; j< 4; j++)
            {
                std::cout << std::fixed << std::setprecision(6) << trans(i,j) << " , ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::this_thread::sleep_until(next_time);
        t += dt;
    }
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    fprintf(fwrite_data, "{");
    for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
    {
        if (!robot_config.motors[calibration_arm_index].enabled[j]) continue;
        fprintf(fwrite_data, "%f, ",  DH_pos[calibration_arm_index][j]);
    }
    
    fprintf(fwrite_data2, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
            trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3),
            trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3),
            trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
    
    fprintf(fwrite_data, "},\n");
    fprintf(fwrite_data2, "\n");
    fclose(fwrite_data2);
    fclose(fwrite_data);
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}