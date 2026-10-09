#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "config_loader.h"
#include <math.h>
#include "usb_to_can.h"
#include <callback_dispatcher.h>

using json = nlohmann::json;
namespace fs = std::filesystem;

bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}

json read_json_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }
    json data;
    file >> data;
    return data;
}

void save_json_file(const json& data, const std::string& filename) {
    // 确保目录存在
    fs::path path(filename);
    fs::create_directories(path.parent_path());
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法创建文件: " + filename);
    }
    file << data.dump(4);
}

// 处理单个机械臂的FK计算
void process_arm_fk(json& step, const std::string& arm_name, int arm_index) {
    if (step.contains("states") && step["states"].contains(arm_name)) {
        auto& arm_state = step["states"][arm_name];
        
        if (arm_state.contains("qpos") && arm_state["qpos"].is_array()) {
            auto& qpos_array = arm_state["qpos"];
            
            double DH_pos_rad[NUM_JOINTS_PER_ARM];
            double ee_pos_ori[6];
            
            if (qpos_array.size() == NUM_JOINTS_PER_ARM) {
                // 将qpos转换为弧度（如果当前是弧度则不需要转换）
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j) {
                    DH_pos_rad[j] = qpos_array[j].get<double>();
                    std::cout << DH_pos_rad[j] << " ";

                    // 如果qpos是角度，需要转换为弧度：
                    // DH_pos_rad[j] = qpos_array[j].get<double>() * DEGREE2RAD;
                }
                std::cout << std::endl;
                
                // 执行正向运动学计算
                std::cout << "arm_index: " << arm_index << std::endl;
                kinematics::forwardKinematics(arm_index, DH_pos_rad, ee_pos_ori);
                
                for (int j = 0; j < 3; ++j) {
                    ee_pos_ori[j] /= 1000.0;  // mm to m
                }
                
                // 更新eef_pose字段
                if (!arm_state.contains("eef_pose") || 
                    !arm_state["eef_pose"].is_array() ||
                    arm_state["eef_pose"].size() != 6) {
                    arm_state["eef_pose"] = json::array();
                }
                
                auto& eef_pose_array = arm_state["eef_pose"];
                for (int j = 0; j < 6; ++j) {
                    if (j < eef_pose_array.size()) {
                        eef_pose_array[j] = ee_pos_ori[j];
                    } else {
                        eef_pose_array.push_back(ee_pos_ori[j]);
                    }
                }
                
                std::cout << arm_name << " 索引 " << step["idx"] << " - FK完成: [";
                for (int j = 0; j < 6; ++j) {
                    std::cout << std::fixed << std::setprecision(6) << ee_pos_ori[j];
                    if (j < 5) std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            } else {
                std::cerr << "警告: " << arm_name << " 索引 " << step["idx"] 
                          << " 的qpos数组大小不正确: " << qpos_array.size() 
                          << " (期望: " << NUM_JOINTS_PER_ARM << ")" << std::endl;
            }
        } else {
            std::cerr << "警告: " << arm_name << " 索引 " << step["idx"] << " 缺少qpos数据" << std::endl;
        }
    } else {
        std::cerr << "警告: 索引 " << step["idx"] << " 缺少 " << arm_name << " 数据" << std::endl;
    }
}

// 处理单个episode文件夹
bool process_episode_folder(const fs::path& episode_path) {
    fs::path input_file = episode_path / "data.json";
    fs::path output_file = episode_path / "data_with_fk.json";
    
    if (!fs::exists(input_file)) {
        std::cerr << "警告: 在 " << episode_path << " 中找不到 data.json 文件" << std::endl;
        return false;
    }
    
    try {
        std::cout << "正在处理episode: " << episode_path.filename().string() << std::endl;
        std::cout << "读取文件: " << input_file << std::endl;
        
        json data = read_json_file(input_file.string());
        
        if (!data.contains("data") || !data["data"].is_array()) {
            std::cerr << "错误: " << input_file << " 文件格式错误：缺少data数组" << std::endl;
            return false;
        }
        
        int total_steps = data["data"].size();
        std::cout << "找到 " << total_steps << " 个时间步数据" << std::endl;
        
        // 处理每个时间步的左右机械臂
        for (auto& step : data["data"]) {
            int idx = step["idx"];
            
            // 处理右臂 (arm_index = 1)
            process_arm_fk(step, "right_arm", 1);

            process_arm_fk(step, "left_arm", 0);

        }
        
        // 保存到新的文件，不覆盖原始文件
        save_json_file(data, output_file.string());
        std::cout << "FK计算完成，结果已保存到: " << output_file << std::endl;
        std::cout << "原始文件 " << input_file << " 保持不变" << std::endl;
        std::cout << "==========================================" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "处理 " << episode_path << " 时出错: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();
    
    // 检查命令行参数
    std::string base_folder;
    if (argc > 1) {
        base_folder = argv[1];
    } else {
        // 如果没有指定文件夹，使用当前目录
        base_folder = ".";
        std::cout << "未指定文件夹，使用当前目录: " << fs::current_path().string() << std::endl;
    }
    
    try {
        fs::path base_path(base_folder);
        
        if (!fs::exists(base_path)) {
            throw std::runtime_error("文件夹不存在: " + base_folder);
        }
        
        if (!fs::is_directory(base_path)) {
            throw std::runtime_error("路径不是文件夹: " + base_folder);
        }
        
        std::cout << "开始扫描文件夹: " << base_path.string() << std::endl;
        
        // 统计信息
        int total_episodes = 0;
        int processed_episodes = 0;
        int failed_episodes = 0;
        
        // 遍历文件夹，查找episode_XXXX子文件夹
        for (const auto& entry : fs::directory_iterator(base_path)) {
            if (entry.is_directory()) {
                std::string folder_name = entry.path().filename().string();
                
                // 检查是否是episode_XXXX格式的文件夹
                if (folder_name.find("episode_") == 0) {
                    total_episodes++;
                    
                    if (process_episode_folder(entry.path())) {
                        processed_episodes++;
                    } else {
                        failed_episodes++;
                    }
                }
            }
        }
        
        // 输出统计信息
        std::cout << "\n=== 处理完成 ===" << std::endl;
        std::cout << "总episode数量: " << total_episodes << std::endl;
        std::cout << "成功处理: " << processed_episodes << std::endl;
        std::cout << "处理失败: " << failed_episodes << std::endl;
        
        if (total_episodes == 0) {
            std::cout << "警告: 没有找到episode_XXXX格式的文件夹" << std::endl;
            std::cout << "请确保文件夹名称格式为: episode_0001, episode_0002, ..." << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "使用方法: " << argv[0] << " [文件夹路径]" << std::endl;
        std::cerr << "如果不指定文件夹路径，将使用当前目录" << std::endl;
        return 1;
    }
    
    return 0;
}