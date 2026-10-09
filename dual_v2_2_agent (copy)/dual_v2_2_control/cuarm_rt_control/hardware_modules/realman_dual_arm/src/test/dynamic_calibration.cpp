#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <algorithm>
#include "robot_arm.h"
#include "config_loader.h"
#include "kinematics.h"
#include "can_fd.h"

// 常量定义
constexpr double MAX_TIME = 30.0;
constexpr int ITPV = 10; // 10ms采样间隔
constexpr double DT = ITPV / 1000.0; // 时间步长（秒）
constexpr int NUM_TRAJ_COLUMNS = 22; // 22行数据

using Trajectory = std::vector<std::vector<double>>;

// 正确的数据读取函数 - 按22行格式读取
bool readTrajectoryFromFile(const std::string& filename, Trajectory& traj) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return false;
    }

    traj.clear();
    traj.resize(NUM_TRAJ_COLUMNS);
    
    std::string line;
    int row_index = 0;
    
    // 读取22行数据
    while (std::getline(file, line) && row_index < NUM_TRAJ_COLUMNS) {
        std::stringstream ss(line);
        std::string value_str;
        
        while (std::getline(ss, value_str, ',')) {
            try {
                if (!value_str.empty()) {
                    traj[row_index].push_back(std::stod(value_str));
                }
            } catch (const std::exception& e) {
                std::cerr << "Error converting value: " << value_str << std::endl;
            }
        }
        
        std::cout << "Row " << row_index << " has " << traj[row_index].size() << " data points" << std::endl;
        row_index++;
    }
    
    file.close();
    
    // 检查数据完整性
    if (traj[0].empty()) {
        std::cerr << "Error: No data read from file" << std::endl;
        return false;
    }
    
    std::cout << "Trajectory loaded successfully. Total time points: " << traj[0].size() << std::endl;
    return true;
}

void followTrajectory(RobotArms& arm, int arm_index, const Trajectory& ref_traj,
                      std::vector<double>& save_count, std::vector<double>& save_time_linux,
                      std::vector<std::vector<double>>& save_q, std::vector<std::vector<double>>& save_dq,
                      std::vector<std::vector<double>>& save_tau, std::vector<std::vector<double>>& save_qr,
                      std::vector<std::vector<double>>& save_dqr, std::vector<std::vector<double>>& save_ddqr,
                      std::vector<std::vector<double>>& save_torsen,std::vector<std::vector<double>>& save_ref) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    int count = 0;
    bool last_sample_flag = false;
    const int max_points = ref_traj[0].size();

    std::cout << "Starting trajectory following with " << max_points << " points" << std::endl;

    while (count < max_points && !last_sample_flag) {
        auto now = std::chrono::high_resolution_clock::now();
        double cur_time_s = std::chrono::duration<double>(now - start_time).count();

        // 检查轨迹结束（老代码逻辑）
        if (count > 0 && ref_traj[21][count] - ref_traj[21][count - 1] < 0) {
            last_sample_flag = true;
            std::cout << "Trajectory end detected at count: " << count << std::endl;
            break;
        }

        // 获取目标轨迹（老代码格式）
        std::vector<double> target_pos(NUM_JOINTS_PER_ARM), target_vel(NUM_JOINTS_PER_ARM), target_acc(NUM_JOINTS_PER_ARM);
        
        for (int joint = 0; joint < NUM_JOINTS_PER_ARM; ++joint) {
            if (last_sample_flag) {
                // 使用最后一个有效点
                target_pos[joint] = ref_traj[joint][count - 1];
                target_vel[joint] = 0.0;
                target_acc[joint] = 0.0;
            } else {
                // 正常读取轨迹点
                target_pos[joint] = ref_traj[joint][count];
                target_vel[joint] = ref_traj[joint + 7][count];
                target_acc[joint] = ref_traj[joint + 14][count];
            }
        }

        // 设置机械臂位置（转换为度）
        for (int joint = 0; joint < NUM_JOINTS_PER_ARM; ++joint) {
            arm.set_mdh_position(joint, target_pos[joint] * RAD2DEGREE);
        }

        // 读取实际状态
        std::vector<double> act_q(NUM_JOINTS_PER_ARM), act_dq(NUM_JOINTS_PER_ARM), 
                           act_cur(NUM_JOINTS_PER_ARM), act_tau(NUM_JOINTS_PER_ARM);
        
        for (int joint = 0; joint < NUM_JOINTS_PER_ARM; ++joint) {
            act_q[joint] = arm.get_mdh_position(joint);        // 度
            act_dq[joint] = arm.get_velocity(joint);           // 度/秒
            act_cur[joint] = arm.get_current(joint);           // 电流
            
            // 读取扭矩传感器数据
            if (arm.sensor_enable_list_[joint] == 1) {
                arm.read_torque_sensor(joint);
                act_tau[joint] = arm.get_torque_sensor_data(joint);
            } else {
                act_tau[joint] = 0.0;
            }
        }

        // 保存数据（仅在轨迹未结束时）
        if (!last_sample_flag) {

            save_count.push_back(count);
            save_time_linux.push_back(cur_time_s);
            save_q.push_back(act_q);
            save_dq.push_back(act_dq);
            save_tau.push_back(act_cur);
            save_qr.push_back(target_pos);
            save_dqr.push_back(target_vel);
            save_ddqr.push_back(target_acc);
            save_torsen.push_back(act_tau);
        }

        // 等待下一个周期（10ms）
        auto next_time = now + std::chrono::milliseconds(ITPV);
        std::this_thread::sleep_until(next_time);
        
        count++;
        
        // 进度显示
        if (count % 100 == 0) {
            std::cout << "Processed " << count << "/" << max_points << " points" << std::endl;
        }
    }
    
    std::cout << "Trajectory following completed. Total points processed: " << count << std::endl;
}

void writeDataToFile(const std::string& filename, const std::vector<double>& save_count,
                     const std::vector<double>& save_time_linux, const std::vector<std::vector<double>>& save_q,
                     const std::vector<std::vector<double>>& save_dq, const std::vector<std::vector<double>>& save_tau,
                     const std::vector<std::vector<double>>& save_qr, const std::vector<std::vector<double>>& save_dqr,
                     const std::vector<std::vector<double>>& save_ddqr, const std::vector<std::vector<double>>& save_torsen) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open output file " << filename << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6);
    
    for (size_t cnt = 0; cnt < save_count.size(); ++cnt) {
        // 计数和时间
        file << save_time_linux[cnt] << " ";
        
        // 实际位置（7个关节）
        for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
            file << save_q[cnt][i] << " ";
        }
        
        // 实际速度（7个关节）
        for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
            file << save_dq[cnt][i] << " ";
        }
        
        // 实际电流/扭矩（7个关节）
        for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
            file << save_tau[cnt][i] << " ";
        }
        
        // 目标位置（7个关节）
        for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
            file << save_qr[cnt][i] << " ";
        }
        
        // 目标速度（7个关节）
        for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
            file << save_dqr[cnt][i] << " ";
        }
        
        // 目标加速度（7个关节）
        for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
            file << save_ddqr[cnt][i] << " ";
        }
        
        // 扭矩传感器数据（7个关节）
        for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
            file << save_torsen[cnt][i] << " ";
        }
        
        file << std::endl;
    }
    
    file.close();
    std::cout << "Data written to " << filename << ", total points: " << save_count.size() << std::endl;
}

int main() {
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robots = config_loader.getRobots();
    // 初始化CAN
    canfd_init(robots[0].motors.can_channel, 0);
    canfd_init(robots[1].motors.can_channel, 0);
    can_normal_init(robots[0].torque_sensors.can_channel, EU_CANABLE);
    can_normal_init(robots[1].torque_sensors.can_channel, EU_CANABLE);
    // 初始化机械臂
    RobotArms arms[NUM_ARM];
    int arm_index = 0; // 0表示左臂，1表示右臂
    int joint = 6; // 0-6表示第1-7个关节
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arms[i]) RobotArms(i, robots[i]);
        if (arms[i].initialize() != 0) return -1;
        if (arms[i].set_control_mode(pos_control) != 0) return -2;
    }

    // 读取轨迹数据
    std::string traj_filename = "/home/xyd/rt_ws/dual_arm_v2/dualarmv2_data_read_grav_fric_ident_J_test" + std::to_string(joint) + ".txt";
    
    Trajectory ref_traj;
    if (!readTrajectoryFromFile(traj_filename, ref_traj)) {
        std::cerr << "Failed to read trajectory file" << std::endl;
        return 1;
    }

    // 准备数据存储
    std::vector<double> save_count, save_time_linux;
    std::vector<std::vector<double>> save_q, save_dq, save_tau, save_qr, save_dqr, save_ddqr, save_torsen, save_ref;

    // 跟随轨迹
    followTrajectory(arms[arm_index], arm_index, ref_traj, save_count, save_time_linux, 
                    save_q, save_dq, save_tau, save_qr, save_dqr, save_ddqr, save_torsen, save_ref);

    // 写入结果文件
    std::string output_filename = "arm_" + std::to_string(arm_index) + "_joint_" + std::to_string(joint) + "_results.txt";
    writeDataToFile(output_filename, save_count, save_time_linux, save_q, save_dq, save_tau, 
                   save_qr, save_dqr, save_ddqr, save_torsen);

    std::cout << "Program completed successfully" << std::endl;
    return 0;
}