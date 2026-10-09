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
std::vector<std::vector<float>> read_txt_file(std::string path, int start_index, int size){
    int n = size;
    std::vector<std::vector<float>> trajectory;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        std::vector<float> row;
        std::stringstream ss(line);
        float num;
        
        while (ss >> num) {
            row.push_back(num);
        }
        
        std::vector<float> point(n);
        for (int i=0; i<n; i++){
            point[i] = row[start_index+i];
        }
        trajectory.push_back(point);
    }
    file.close();
    return trajectory;
}

void convert_trajectory_to_degree(std::vector<std::vector<float>>& trajectory){
    for (int i=0; i<trajectory.size(); i++){
        for (int j=0; j<trajectory[i].size(); j++){
            trajectory[i][j] = trajectory[i][j] * RAD2DEGREE;
        }
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
    std::ofstream writer("data.txt");
    double act_pos[NUM_ARM][NUM_JOINTS_PER_ARM], act_vel[NUM_ARM][NUM_JOINTS_PER_ARM], act_cur[NUM_ARM][NUM_JOINTS_PER_ARM];
    double pos[NUM_ARM][NUM_JOINTS_PER_ARM];

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
    std::vector<std::vector<float>> trajectory = read_txt_file("/ssd/eyou_dual_arm/build/data1.txt", 84, 14);
    convert_trajectory_to_degree(trajectory);
    int start_row = 1200;
    int n = trajectory.size();

    //Go to start position
    std::cout<<"Go to start position"<<std::endl;
    go_to_position(arm, robot_config, trajectory[start_row]);
    
    std::cout<<"Start trajectory"<<std::endl;
    int delay_ms = robot_config.control.position[0].dt * 1e3;
    // int delay_ms = 0.1 * 1e3;
    for (int row = start_row; row < n; row++)
    {   
        std::cout << "row: " << row << std::endl;
        for (int i = 0;i < NUM_ARM;++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                pos[i][j] = trajectory[row][i*NUM_JOINTS_PER_ARM + j];
                arm[i].set_mdhpos(j, pos[i][j]);
            }

            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                act_pos[i][j] = arm[i].get_mdhpos(j);
                act_vel[i][j] = arm[i].get_velocity(j) * robot_config.motors[i].sign[j] / robot_config.motors[i].gear_ratios[j];
                act_cur[i][j] = arm[i].get_current(j);
                writer << pos[i][j] * DEGREE2RAD << " " << act_pos[i][j] * DEGREE2RAD << " "<< act_vel[i][j]<< " "<< act_cur[i][j]<< " ";     
            }
        }
        writer << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    
    float wait_ms = delay_ms * 1;
    int cycle_cnt = 0;
    int total_cycle = wait_ms / delay_ms;
    printf("Record data for additional %f second\n", wait_ms / 1e3);
    while (cycle_cnt < total_cycle){
        for (int i = 0;i < NUM_ARM;++i)
        {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
            {
                if (!robot_config.motors[i].enabled[j]) continue;
                act_pos[i][j] = arm[i].get_mdhpos(j);
                act_vel[i][j] = arm[i].get_velocity(j) / robot_config.motors[i].gear_ratios.data()[j];
                act_cur[i][j] = arm[i].get_current(j);
                writer << pos[i][j] * DEGREE2RAD << " " << act_pos[i][j] * DEGREE2RAD << " "<< act_vel[i][j]<< " "<< act_cur[i][j]<< " ";     
            }
        }
        writer << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        cycle_cnt ++;
    }

    // // //////////////////////////////////////Follow Trajectory
    
    writer.close();
    arm[0].all_motor_stop();
    arm[1].all_motor_stop();
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}
