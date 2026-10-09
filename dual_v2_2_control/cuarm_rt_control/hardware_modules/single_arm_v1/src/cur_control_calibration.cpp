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

bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}

// 设置非阻塞键盘输入
int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

// 弧度转角度常量（如果未在头文件中定义）
#ifndef RAD2DEG
const double RAD2DEG = 180.0 / M_PI;
#endif
#ifndef DEGREE2RAD
const double DEGREE2RAD = M_PI / 180.0;
#endif

int main()
{
    signal(SIGINT, signal_handler);

    // 加载配置文件（单臂配置）
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto &robot_config = config_loader.getRobot(); // 获取单臂配置

    // 初始化CAN（使用新示例的方式）
    can_normal_init(0);
    canfd_init(0, 0);

    // 创建并初始化单臂
    RobotArm single_arm(robot_config);
    if (single_arm.initialize(100) != 0) // 100 可能为超时或重试次数
    {
        std::cerr << "Failed to initialize robot arm" << std::endl;
        return -1;
    }
    if (single_arm.set_control_mode(cur_control) != 0) // cur_control 需已在 robot_arm.h 中定义
    {
        std::cerr << "Failed to set control mode" << std::endl;
        return -2;
    }

    // 打开文件保存关节角度数据
    FILE *fwrite_data = fopen("joint_pos_data.txt", "w");
    if (fwrite_data == nullptr)
    {
        std::cerr << "Failed to open joint_pos_data.txt for writing" << std::endl;
        return -3;
    }

    std::cout << "程序开始运行..." << std::endl;
    std::cout << "按下回车键将存储当前关节角度到文件，按Ctrl+C退出程序" << std::endl;

    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double dt = 0.01; // 控制周期 10ms
    int T = 3600;     // 运行总时间 3600 秒
    double t = dt;    // 当前时间
    int NUM_JOINTS_PER_ARM=single_arm.num_joints_;
    // 存储关节角度（度）和弧度值
    double DH_pos[NUM_JOINTS_PER_ARM];
    double DH_pos_rad[NUM_JOINTS_PER_ARM];

    while (!exit_requested && t < T)
    {
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));

        // 检测回车键
        if (kbhit())
        {
            char c = getchar();
            if (c == '\n')
            {
                // 将当前关节角度（度）写入文件
                fprintf(fwrite_data, "{");
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
                {
                    if (!robot_config.motors.enabled[j])
                        continue;
                    fprintf(fwrite_data, "%f, ", DH_pos[j]);
                }
                fprintf(fwrite_data, "},\n");
                fflush(fwrite_data);
                std::cout << "关节角度已保存到文件!" << std::endl;
            }
        }

        // 读取所有关节状态
        for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j)
        {
            if (!robot_config.motors.enabled[j])
                continue;

            // 设置电流为0（保持原代码行为）
            single_arm.set_current(j, 0);

            // 显式读取关节状态（确保数据最新）
            single_arm.read_status(j);

            // 获取关节角度（假设 get_mdh_position() 返回弧度）
            double pos_rad = single_arm.get_mdh_position(j);
            DH_pos[j] = pos_rad * RAD2DEG; // 转换为度用于保存
            DH_pos_rad[j] = pos_rad;

            // 每10个周期打印一次信息（与原代码逻辑一致）
            if ((int)(t / dt) % 10 == 0)
            {
                std::cout << "Joint " << j
                          << " | MDH pos (deg) " << DH_pos[j]
                          << std::endl;
            }
        }

        std::this_thread::sleep_until(next_time);
        t += dt;
    }

    // 清理
    fclose(fwrite_data);
    can_normal_close(0);
    canfd_close(0, 0);

    std::cout << "程序结束" << std::endl;
    return 0;
}