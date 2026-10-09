#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "config_loader.h"
#include "can_fd.h"
#include "can_normal.h"
#include "robot_arm.h"
#include "kinematics.h"
#include "robotics_math.h"


// 弧度转角度常量（如果未在头文件中定义）
#ifndef RAD2DEG
const double RAD2DEG = 180.0 / M_PI;
#endif
#ifndef DEGREE2RAD
const double DEGREE2RAD = M_PI / 180.0;
#endif
int main(int argc, char *argv[])
{
    // Load configuration from config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto &robot_config = config_loader.getRobot();
    

    // 初始化CAN
    can_normal_init(0);
    canfd_init(0, 0);
    RobotArm single_arm_v1(robot_config);
    single_arm_v1.initialize(100);
    single_arm_v1.set_control_mode(pos_control);
         int NUM_JOINTS_PER_ARM=single_arm_v1.num_joints_;
         // 存储关节角度（度）和弧度值
    double DH_pos[NUM_JOINTS_PER_ARM];
    double DH_pos_rad[NUM_JOINTS_PER_ARM];
    // 读取所有关节状态
    for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
    {
        if (!robot_config.motors.enabled[j])
            continue;

  
        // 显式读取关节状态（确保数据最新）
        single_arm_v1.read_status(j);

        // 获取关节角度（假设 get_mdh_position() 返回弧度）
        double pos_rad = single_arm_v1.get_mdh_position(j);
        DH_pos[j] = pos_rad * RAD2DEG; // 转换为度用于保存
        DH_pos_rad[j] = pos_rad;

    }
    printf("DH_pos_rad:%f,%f,%f,%f,%f,%f ",DH_pos_rad[0],DH_pos_rad[1],DH_pos_rad[2],DH_pos_rad[3],DH_pos_rad[4],DH_pos_rad[5]);
    double ee_pos_ori[6];
    kinematics::forwardKinematics(robot_config,DH_pos_rad,ee_pos_ori);
    printf("ee_pos_ori:%f,%f,%f,%f,%f,%f ",ee_pos_ori[0],ee_pos_ori[1],ee_pos_ori[2],ee_pos_ori[3],ee_pos_ori[4],ee_pos_ori[5]);
   
    const int num_points =5;
    double target_dest[num_points][6] = {
        {ee_pos_ori[0],ee_pos_ori[1],ee_pos_ori[2],ee_pos_ori[3],ee_pos_ori[4],ee_pos_ori[5]},
        {ee_pos_ori[0],ee_pos_ori[1],ee_pos_ori[2]-100,ee_pos_ori[3],ee_pos_ori[4],ee_pos_ori[5]},
        {ee_pos_ori[0]+100,ee_pos_ori[1],ee_pos_ori[2]-100,ee_pos_ori[3],ee_pos_ori[4],ee_pos_ori[5]},
        {ee_pos_ori[0]+100,ee_pos_ori[1],ee_pos_ori[2],ee_pos_ori[3],ee_pos_ori[4],ee_pos_ori[5]},
        {ee_pos_ori[0],ee_pos_ori[1],ee_pos_ori[2],ee_pos_ori[3],ee_pos_ori[4],ee_pos_ori[5]}
    };
    double dt = 0.01;
    double T = 5.0;
    for(int i = 0; i < num_points; i++){
        double target_temp[6];
        memcpy(target_temp, target_dest[i], sizeof(target_temp));
        single_arm_v1.move_line_online(target_temp,dt,T);
    }

    return 0;
}
