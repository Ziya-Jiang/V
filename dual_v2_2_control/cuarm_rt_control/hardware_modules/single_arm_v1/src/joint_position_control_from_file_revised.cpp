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
#include <sstream>
#include <algorithm>
#include <cctype>
#include "robot_arm.h" // 单臂类头文件（应包含 RobotArm 定义）
#include "config_loader.h"
#include "can_fd.h"
#include "can_normal.h" // 新增普通 CAN 初始化
#include "polynomial_trajectory.h"

int NUM_JOINTS_PER_ARM = 6;
// 弧度转角度常量（如果未在头文件中定义）
#ifndef RAD2DEG
const double RAD2DEG = 180.0 / M_PI;
#endif
#ifndef DEGREE2RAD
const double DEGREE2RAD = M_PI / 180.0;
#endif

// 辅助函数：去除字符串中的特殊字符（保持不变）
std::string clean_line(const std::string &line)
{
    std::string result;
    for (char c : line)
    {
        if (c == '{' || c == '}' || c == ',')
        {
            result += ' ';
        }
        else
        {
            result += c;
        }
    }
    return result;
}

// 读取 txt 文件的函数，处理花括号和逗号（保持不变）
std::vector<std::vector<float>> read_txt_file(std::string path)
{
    std::vector<std::vector<float>> data;
    std::ifstream file(path);
    std::string line;

    while (std::getline(file, line))
    {
        std::string clean = clean_line(line);
        std::stringstream ss(clean);
        std::vector<float> row;
        float num;

        while (ss >> num)
        {
            row.push_back(num);
        }

        if (!row.empty() && row.size() >= 6)
        {
            if (row.size() > 7)
            {
                row.resize(7);
            }
            data.push_back(row);
        }
    }

    file.close();
    std::cout << "Read " << data.size() << " valid rows from file." << std::endl;
    return data;
}

// 单个机械臂移动到目标位置（改为接受 RobotArm 类型）
void single_arm_go_to_position(RobotArm &arm, const Robot &robot_config,
                               std::vector<float> target_position)
{
    std::vector<double> traj_coef[NUM_JOINTS_PER_ARM];
    double pos, vel, acc;

    std::cout << "Target position: ";
    for (int j = 0; j < target_position.size() && j < NUM_JOINTS_PER_ARM; ++j)
    {
        std::cout << target_position[j] << " ";
    }
    std::cout << std::endl;

    double dt = robot_config.control.position.dt; // 单臂配置中路径
    double trajectory_time = robot_config.control.position.T;
    double t = 0;

    // 计算每个关节的轨迹系数
    for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
    {
        if (!robot_config.motors.enabled[j])
            continue;

        double target_pos = target_position[j];
        double start_pos = arm.get_mdh_position(j); // 假设返回弧度

        std::cout << "Joint " << j << ": start=" << start_pos
                  << ", target=" << target_pos
                  << ", time=" << trajectory_time << "s" << std::endl;

        traj_coef[j] = polynomial_trajectory::calculateCoefficients(
            start_pos, 0.0, 0.0, target_pos, 0.0, 0.0, trajectory_time);
    }

    // 执行轨迹
    int num_steps = static_cast<int>(trajectory_time / dt);
    for (int step = 0; step <= num_steps; ++step)
    {
        t = step * dt;
        if (t > trajectory_time)
            t = trajectory_time;

        for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
        {
            if (!robot_config.motors.enabled[j])
                continue;

            std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[j], t);
            arm.set_mdh_position(j, pos); // 假设单臂有该方法
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Arm reached target position" << std::endl;
}

int main(int argc, char *argv[])
{
    // // 打开文件保存末端执行器位置
    // FILE *fwrite_data = fopen("end_effector_positions.txt", "w");
    // if (!fwrite_data)
    // {
    //     std::cerr << "Failed to open end_effector_positions.txt for writing" << std::endl;
    //     return -1;
    // }

 

    double act_pos[NUM_JOINTS_PER_ARM]; // 单臂关节角度（弧度）
    double ee_pos_ori[6];               // 末端位姿
    // matrix::Matrix<double, 4UL, 4UL> trans;

    // 从 config.yaml 加载单臂配置
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto &robot_config = config_loader.getRobot(); // 单臂配置

    // 初始化 CAN（普通 CAN 和 CAN FD）
    can_normal_init(0);
    canfd_init(0, 0);

    // 创建并初始化单臂
    RobotArm single_arm(robot_config);
    if (single_arm.initialize(100) != 0)
    {
        std::cerr << "Failed to initialize robot arm" << std::endl;
        return -1;
    }
    if (single_arm.set_control_mode(pos_control) != 0)
    { // pos_control 应在头文件中定义
        std::cerr << "Failed to set control mode" << std::endl;
        return -2;
    }

    // 从文件读取轨迹点（每个点包含6个关节角度，单位应为度）
    std::vector<std::vector<float>> trajectory_deg = read_txt_file("joint_pos_data.txt");
    if (trajectory_deg.empty())
    {
        std::cerr << "No trajectory data loaded!" << std::endl;
        return -3;
    }

    // 将度数转换为弧度（假设DEGREE2RAD已定义为 M_PI/180.0）
    std::vector<std::vector<float>> trajectory;
    trajectory.reserve(trajectory_deg.size());
    for (const auto &point_deg : trajectory_deg)
    {
        std::vector<float> point_rad;
        point_rad.reserve(point_deg.size());
        for (float val : point_deg)
        {
            point_rad.push_back(val * DEGREE2RAD);
        }
        trajectory.push_back(point_rad);
    }

    std::cout << "Starting trajectory replay with " << trajectory.size() << " points" << std::endl;

    int delay_between_points = 5000; // 点之间的延迟（毫秒）

    // 执行轨迹
    for (int row = 0; row < trajectory.size(); ++row)
    {
        std::cout << "\n=== Moving to point " << row << " ===" << std::endl;

        // 移动到目标位置（传入单臂对象和配置）
        single_arm_go_to_position(single_arm, robot_config, trajectory[row]);

        // 获取当前实际位置（弧度）
        for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
        {
            if (!robot_config.motors.enabled[j])
                continue;
            act_pos[j] = single_arm.get_mdh_position(j); // 假设返回弧度
        }

        // // 计算正运动学（臂索引固定为 0）
        // kinematics::forwardKinematics(0, act_pos, ee_pos_ori);

        // 显示关节位置（转换为度）
        std::cout << "Actual joint positions (degrees): ";
        for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
        {
            if (!robot_config.motors.enabled[j])
                continue;
            std::cout << act_pos[j] * RAD2DEG << " ";
        }
        std::cout << std::endl;

        // // 显示末端执行器位置
        // std::cout << "End effector position/orientation: ";
        // for (int i = 0; i < 3; ++i)
        // {
        //     std::cout << ee_pos_ori[i] << " ";
        // }
        // std::cout << std::endl;

        // // 转换为变换矩阵
        // robotics_math::Eula2Mat(trans, ee_pos_ori);

        // // 保存末端执行器位置到文件（变换矩阵 12 个元素）
        // fprintf(fwrite_data, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
        //         trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3),
        //         trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3),
        //         trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
        // fflush(fwrite_data);

        // 点之间的延迟
        std::cout << "Waiting " << delay_between_points << "ms before next point..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_between_points));
    }

    std::cout << "\nTrajectory replay completed!" << std::endl;

    // 清理
    // single_arm.disable();
    // fclose(fwrite_data);
    can_normal_close(0);
    canfd_close(0, 0);

    return 0;
}