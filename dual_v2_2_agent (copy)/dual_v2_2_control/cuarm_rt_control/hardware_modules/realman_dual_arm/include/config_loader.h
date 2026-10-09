#pragma once
#include <yaml-cpp/yaml.h>
#include <array>
#include <stdexcept>
#include <string>
#include <fstream>
#include "robot_struct.h"  // 包含整合后的Robot

class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& filename);
    const std::array<Robot, NUM_ARM>& getRobots() const { return robots_; }

private:
    std::array<Robot, NUM_ARM> robots_;

    void loadFromFile(const std::string& filename);
    void parseKinematics(const YAML::Node& node, int arm_index);
    void parseDynamics(const YAML::Node& node, int arm_index);
    void parseHardware(const YAML::Node& node, int arm_index);
    void parseControl(const YAML::Node& node, int arm_index);

    template<typename T, size_t N>
    void copyYAMLToArray(const YAML::Node& yaml_node, std::array<T, N>& dest) {
        if (!yaml_node || yaml_node.size() > N) {
            throw std::runtime_error("Invalid array size in YAML or node is empty");
        }
        for (size_t i = 0; i < yaml_node.size(); ++i) {
            dest[i] = yaml_node[i].as<T>();
        }
    }

    template<typename T, size_t N>
    void copyYAMLToArrayOpt(const YAML::Node& yaml_node, std::array<T, N>& dest, bool optional = false) {
        if (!yaml_node && optional) return;
        if (!yaml_node || yaml_node.size() != N) {
            throw std::runtime_error("Invalid or missing array in YAML");
        }
        for (size_t i = 0; i < N; ++i) {
            dest[i] = yaml_node[i].as<T>();
        }
    }
};

ConfigLoader::ConfigLoader(const std::string& filename) {
    loadFromFile(filename);
}

void ConfigLoader::loadFromFile(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);

    if (!config["robot"] || !config["kinematics"] || !config["hardware"] || !config["control"]) {
        throw std::runtime_error("Missing required sections in config file");
    }

    // 加载每个臂的配置
    for (int i = 0; i < NUM_ARM; ++i) {
        parseKinematics(config["kinematics"], i);
        parseDynamics(config["dynamics"], i);
        parseHardware(config["hardware"], i);
        parseControl(config["control"], i);
    }
}

void ConfigLoader::parseKinematics(const YAML::Node& node, int arm_index) {
    const char* arm_names[NUM_ARM] = {"left_arm", "right_arm"};
    auto& robot = robots_[arm_index];
    const YAML::Node& arm_node = node[arm_names[arm_index]];

    copyYAMLToArray(arm_node["a"], robot.mdh.a);
    copyYAMLToArray(arm_node["alpha"], robot.mdh.alpha);
    copyYAMLToArray(arm_node["d"], robot.mdh.d);
    copyYAMLToArray(arm_node["sign"], robot.mdh.sign);
    copyYAMLToArray(arm_node["base_to_world"], robot.mdh.base_to_world);
    copyYAMLToArray(arm_node["tool"], robot.mdh.tool);
    copyYAMLToArray(arm_node["offset"], robot.mdh.offset);
    copyYAMLToArray(arm_node["init_mdhpos"], robot.mdh.m_init_mdhpos);
    copyYAMLToArray(arm_node["limits"]["max"], robot.joint_limit.joint_positive_limit);
    copyYAMLToArray(arm_node["limits"]["min"], robot.joint_limit.joint_negative_limit);
}

void ConfigLoader::parseDynamics(const YAML::Node& node, int arm_index) {
    const char* arm_names[NUM_ARM] = {"left_arm", "right_arm"};
    auto& robot = robots_[arm_index];
    const YAML::Node& arm_node = node[arm_names[arm_index]];

    copyYAMLToArray(arm_node["dynamics_parm"], robot.dynamic.dynamic_parameters);
}

void ConfigLoader::parseHardware(const YAML::Node& node, int arm_index) {
    const char* arm_names[NUM_ARM] = {"left_arm", "right_arm"};
    auto& robot = robots_[arm_index];
    const YAML::Node& motor_node = node["motor"][arm_names[arm_index]];

    copyYAMLToArray(motor_node["ids"], robot.motors.ids);
    copyYAMLToArray(motor_node["enabled"], robot.motors.enabled);
    copyYAMLToArray(motor_node["gear_ratios"], robot.motors.gear_ratios);
    copyYAMLToArray(motor_node["rated_torque"], robot.motors.rated_torque);
    copyYAMLToArray(motor_node["max_velocity_rpm"], robot.motors.max_velocity_rpm);
    copyYAMLToArray(motor_node["max_current_mA"], robot.motors.max_current_mA);
    copyYAMLToArray(motor_node["max_position_step"], robot.motors.max_position_step);
    robot.motors.use_pdo = node["can"]["use_pdo"].as<bool>();
    robot.motors.can_channel = node["can"][arm_names[arm_index]]["channel"].as<int>();
    robot.motors.pdo_auto_report_ms = node["can"][arm_names[arm_index]]["pdo_auto_report_ms"].as<int>();

    // 扭矩传感器
    const YAML::Node& ts_node = node["torque_sensor"][arm_names[arm_index]];
    copyYAMLToArray(ts_node["ids"], robot.torque_sensors.ids);
    copyYAMLToArray(ts_node["enabled"], robot.torque_sensors.enabled);
    copyYAMLToArray(ts_node["sign"], robot.torque_sensors.sign);
    robot.torque_sensors.can_channel = node["can"]["torque_sensor"]["channel"][arm_index].as<int>();
    robot.torque_sensors.device_type = node["can"]["torque_sensor"]["device_type"].as<int>();

    // 手部
    robot.hands.id = node["hand"][arm_names[arm_index]]["id"].as<int>();
    copyYAMLToArray(node["hand"][arm_names[arm_index]]["cmd"], robot.hands.hand_cmd);

    // 比率复制到ratio
}

void ConfigLoader::parseControl(const YAML::Node& node, int arm_index) {
    const char* arm_names[NUM_ARM] = {"left_arm", "right_arm"};
    auto& robot = robots_[arm_index];
    const YAML::Node& pos_node = node["position"][arm_names[arm_index]];
    const YAML::Node& vel_node = node["velocity"][arm_names[arm_index]];
    const YAML::Node& force_node = node["force"];

    // Position
    robot.control.position[arm_index].dt = pos_node["dt"].as<double>();
    robot.control.position[arm_index].T = pos_node["T"].as<double>();
    robot.control.position[arm_index].use_torque_sensor = pos_node["use_torque_sensor"].as<bool>();
    copyYAMLToArray(pos_node["command_degrees"], robot.control.position[arm_index].command_degrees);
    copyYAMLToArray(pos_node["cartesian_command"], robot.control.position[arm_index].cartesian_command);

    // Velocity
    robot.control.velocity[arm_index].interpolation_dt = vel_node["interpolation_dt"].as<double>();
    robot.control.velocity[arm_index].interpolation_T = vel_node["interpolation_T"].as<int>();
    robot.control.velocity[arm_index].error_tolerance = vel_node["error_tolerance"].as<double>();
    robot.control.velocity[arm_index].use_torque_sensor = vel_node["use_torque_sensor"].as<bool>();
    copyYAMLToArray(vel_node["kp"], robot.control.velocity[arm_index].kp);
    copyYAMLToArray(vel_node["max_velocity"], robot.control.velocity[arm_index].max_velocity);
    copyYAMLToArray(vel_node["command_degrees"], robot.control.velocity[arm_index].command_degrees);

}


#define MAX_TRAJECTORY_POINTS 100
struct TrajectoryList
{
    int num_joint_points;
    int num_cartesian_points;
    double dt[2];
    double T[2];
    double joint_traj_target[NUM_ARM][MAX_TRAJECTORY_POINTS][NUM_JOINTS_PER_ARM];
    double cartesian_target[NUM_ARM][MAX_TRAJECTORY_POINTS][6];
};

class TrajectoryLoader {
public:
    TrajectoryList trajectory;
    
    explicit TrajectoryLoader(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        // 基础参数验证
        parseTrajectoryParams(config["trajectory_list"]);
    }

private:
    void parseTrajectoryParams(const YAML::Node& node) {
        // Parse basic parameters
        trajectory.num_joint_points = node["num_joint_points"].as<int>();
        trajectory.num_cartesian_points = node["num_cart_points"].as<int>();
        // Handle different arm names in YAML (left_arm_t vs left_arm)
        const char* yaml_arm_names[NUM_ARM] = {"left_arm", "right_arm"};
        for (int i = 0; i < NUM_ARM; ++i) {
            const YAML::Node& arm_node = node[yaml_arm_names[i]];
            trajectory.dt[i] = arm_node["dt"].as<double>();
            trajectory.T[i] = arm_node["T"].as<double>();
            for (int point = 0; point < trajectory.num_joint_points; ++point) {
                // std::cout << "arm_node: " << arm_node << std::endl;
                if (!arm_node["joint_angles"] || !arm_node["joint_angles"][point]) {
                    throw std::runtime_error("Invalid or missing 'joint_angles' node in YAML configuration.");
                }
                const YAML::Node& point_data = arm_node["joint_angles"][point];
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j) {
    
                    trajectory.joint_traj_target[i][point][j] = point_data[j].as<double>();
                }
            }

            for (int point = 0; point < trajectory.num_cartesian_points; ++point) {
                const YAML::Node& point_data = arm_node["cartesian_points"][point];
                for (int j = 0; j < 6; ++j) {
                    trajectory.cartesian_target[i][point][j] = point_data[j].as<double>();
                }
            }
            
    
        }
    }
};