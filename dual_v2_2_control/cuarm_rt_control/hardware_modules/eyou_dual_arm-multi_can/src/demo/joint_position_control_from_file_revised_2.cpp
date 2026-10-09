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
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
#include "kinematics.h"
// std::vector<std::vector<float>> read_txt_file(std::string path, int start_index, int size){
//     int n = size;
//     std::vector<std::vector<float>> trajectory;
//     std::ifstream file(path);
//     std::string line;
//     while (std::getline(file, line)) {
//         std::vector<float> row;
//         std::stringstream ss(line);
//         float num;
        
//         while (ss >> num) {
//             row.push_back(num);
//         }
        
//         std::vector<float> point(n);
//         for (int i=0; i<n; i++){
//             point[i] = row[start_index+i];
//         }
//         trajectory.push_back(point);
//     }
//     file.close();
//     return trajectory;
// // }

std::vector<std::vector<float>> read_txt_file(std::string path) {
    std::vector<std::vector<float>> data;
    std::ifstream file(path);
    std::string line;

    while (std::getline(file, line)) {
        std::vector<float> row;
        std::stringstream ss(line);
        float num;

        while (ss >> num) {
            row.push_back(num);
        }

        data.push_back(row);
    }

    file.close();
    return data;
}


void convert_trajectory_to_degree(std::vector<std::vector<float>>& trajectory){
    for (int i=0; i<trajectory.size(); i++){
        for (int j=0; j<trajectory[i].size(); j++){
            trajectory[i][j] = trajectory[i][j] * RAD2DEGREE;
        }
    }
}

void single_arm_go_to_position(EyouJoints *arm, RobotConfig& robot_config, std::vector<float> target_position){

    //Trajectory coefficient generation
    std::vector<double> traj_coef[NUM_ARM][NUM_JOINTS_PER_ARM];
    double pos, vel, acc;
    std::cout << "First row: " << target_position[0] << ", " << target_position[1] << ", " << target_position[2] 
        << target_position[3] << ", " << target_position[4] << ", " << target_position[5] 
        << target_position[6]
        << std::endl; 
    double t[2],  dt[2];
    for (int i = 0; i < NUM_ARM; i++)
    {
        for (int j = 0;j<NUM_JOINTS_PER_ARM; ++j)
        {
            if (!robot_config.motors[i].enabled[j]) continue;
            double target_pos = target_position[j];
            std::cout<< "target_pos joint : " << j << ": "<< target_position[j] << std::endl;
            double start_pos = arm[i].get_mdhpos(j);// target_pos is the target degrees in DH coordinate system  
            traj_coef[i][j] = polynomial_trajectory::calculateCoefficients(start_pos, 0.0, 0.0, target_pos, 0.0, 0.0, robot_config.control.position[i].T);
        }
        dt[i] = robot_config.control.position[i].dt;
        t[i] = dt[i];
    }  

    bool position_reached = false;
    int delay_ms = robot_config.control.position[0].dt * 1e3;
    while (!position_reached)
    { 
        position_reached = true;
        for (int i = 0;i < NUM_ARM;++i)
        {
            if (t[i]<=robot_config.control.position[i].T)
            {
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                {
                    if (!robot_config.motors[i].enabled[j]) continue;
                    std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i][j], t[i]);
                    arm[i].set_mdhpos(j, pos);
                }
                t[i]+=dt[i];
                position_reached = false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}


void go_to_position(EyouJoints *arm, RobotConfig& robot_config, std::vector<float> target_position){
    //Trajectory coefficient generation
    std::vector<double> traj_coef[NUM_ARM][NUM_JOINTS_PER_ARM];
    double pos, vel, acc;

    double t[2],  dt[2];
    for (int i = 0; i < NUM_ARM; i++)
    {
        for (int j = 0;j<NUM_JOINTS_PER_ARM; ++j)
        {
            if (!robot_config.motors[i].enabled[j]) continue;
            double target_pos = target_position[i*NUM_JOINTS_PER_ARM+j];
            double start_pos = arm[i].get_mdhpos(j);// target_pos is the target degrees in DH coordinate system  
            traj_coef[i][j] = polynomial_trajectory::calculateCoefficients(start_pos, 0.0, 0.0, target_pos, 0.0, 0.0, robot_config.control.position[i].T);
        }
        dt[i] = robot_config.control.position[i].dt;
        t[i] = dt[i];
    }  

    bool position_reached = false;
    int delay_ms = robot_config.control.position[0].dt * 1e3;
    while (!position_reached)
    { 
        position_reached = true;
        for (int i = 0;i < NUM_ARM;++i)
        {
            if (t[i]<=robot_config.control.position[i].T)
            {
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                {
                    if (!robot_config.motors[i].enabled[j]) continue;
                    std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i][j], t[i]);
                    arm[i].set_mdhpos(j, pos);
                }
                t[i]+=dt[i];
                position_reached = false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

int main(int argc, char *argv[])
{   
    // std::ofstream writer("data.txt");
    //FILE *fwrite_data = fopen("data.txt", "w"); // save data
    FILE *fwrite_data = fopen("data.txt", "w"); // save data
    FILE *fwrite_joint = fopen("joint_pos.txt", "w"); 
    double act_pos[NUM_ARM][NUM_JOINTS_PER_ARM], act_vel[NUM_ARM][NUM_JOINTS_PER_ARM], act_cur[NUM_ARM][NUM_JOINTS_PER_ARM];
    double pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double ee_pos_ori[NUM_ARM][6];
    matrix::Matrix<double, 4UL, 4UL> trans;

    //Load configuration form config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();

    // Initialize CAN communication and pdo callback dispatcher
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    CallbackDispatcher::init();

    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    EyouJoints arm[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    }
    
    std::cout<< "arm["<<0<<"] can channel: "<<robot_config.motors[0].can_channel<<std::endl;
    arm[0].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[0].pdo_auto_report_ms);
    std::cout<< "arm["<<1<<"] can channel: "<<robot_config.motors[1].can_channel<<std::endl;
    arm[1].init_joints(harmonic_OperateMode_CyclicSyncPosition, robot_config.motors[1].pdo_auto_report_ms);

    //Get trajectory from file
    std::vector<std::vector<float>> trajectory = read_txt_file("joint_pos_data.txt"); // , 0, 3);
    std::cout << "Read " << trajectory.size() << " rows from file." << std::endl;
    std::cout << "First row: " << trajectory[0][0] << ", " << trajectory[0][1] << ", " << trajectory[0][2] 
        << trajectory[0][3] << ", " << trajectory[0][4] << ", " << trajectory[0][5] 
        << trajectory[0][6]
        << std::endl; 
    // convert_trajectory_to_degree(trajectory);
    int start_row = 0;
    int n = trajectory.size();
    int calibration_arm_index = 0; // 0 for left arm, 1 for right arm
    //Go to start position
    // std::cout<<"Go to start position"<<std::endl;
    // go_to_position(arm, robot_config, trajectory[start_row]);
    
    std::cout<<"Start trajectory"<<std::endl;
    int delay_ms = 2000; // robot_config.control.position[0].dt * 1e3;
    // int delay_ms = 0.1 * 1e3;
    for (int row = start_row; row < n; row++)
    {   
        std::cout << "row: " << row << std::endl;

        single_arm_go_to_position(arm, robot_config, trajectory[row]);
        for (int i = 0;i < NUM_ARM;++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                act_pos[i][j] = arm[i].get_mdhpos(j) * DEGREE2RAD ;
                // act_vel[i][j] = arm[i].get_velocity(j) * robot_config.motors[i].sign[j] / robot_config.motors[i].gear_ratios[j];
                // act_cur[i][j] = arm[i].get_current(j);
                // writer << pos[i][j] * DEGREE2RAD << " " << act_pos[i][j] * DEGREE2RAD << " "<< act_vel[i][j]<< " "<< act_cur[i][j]<< " ";     
            }
        }
        for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j) {
            fprintf(fwrite_joint, "%.6f ", act_pos[calibration_arm_index][j] / DEGREE2RAD); 
        }
        fprintf(fwrite_joint, "\n");
        fflush(fwrite_joint);
        kinematics::forwardKinematics(calibration_arm_index, act_pos[calibration_arm_index], ee_pos_ori[calibration_arm_index]);
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < 6; ++i) {
            std::cout << "Joint pos" << i << ": ";
            std::cout << act_pos[calibration_arm_index][i] / DEGREE2RAD << " ";
            std::cout << std::endl;
        }
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < 6; ++i) {
            std::cout << "ee pos" << ": ";
            std::cout << ee_pos_ori[calibration_arm_index][i] << " ";
            std::cout << std::endl;
        }
        robotics_math::Eula2Mat(trans, ee_pos_ori[calibration_arm_index]);
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::cout << trans(i, j) << " ";
            }
            std::cout << std::endl;
        }

        fprintf(fwrite_data, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3),
                trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3),
                trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        fflush(fwrite_data);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
        fclose(fwrite_data);
        fclose(fwrite_joint); 
        arm[0].all_motor_stop();
        arm[1].all_motor_stop();
        canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
        return 0;
}
