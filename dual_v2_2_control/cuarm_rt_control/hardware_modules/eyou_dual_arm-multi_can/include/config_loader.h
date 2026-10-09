#pragma once
#include <yaml-cpp/yaml.h>
#include <array>
#include <cstdint>
#include <stdexcept>
#include "kinematics.h"
#include <string>
#include <fstream>

const size_t NUM_ARM = 2;
const size_t NUM_JOINTS_PER_ARM = 7;
const size_t NUM_JOINTS_HEAD = 2;

// Data Structures
struct KinematicParams {
    std::array<double, NUM_JOINTS_PER_ARM> a{};
    std::array<double, NUM_JOINTS_PER_ARM> alpha{};
    std::array<double, NUM_JOINTS_PER_ARM> d{};
    std::array<int, NUM_JOINTS_PER_ARM> sign{};
    std::array<double, 6> base_to_world{};
    std::array<double, NUM_JOINTS_PER_ARM> offset{};
    std::array<double, NUM_JOINTS_PER_ARM> initial_position{};
};

struct DynamicParms {
    std::array<double, NUM_JOINTS_PER_ARM*4> dynamic_parameters{};
};


struct MotorParams {
    std::array<uint32_t, NUM_JOINTS_PER_ARM> ids{};
    std::array<int, NUM_JOINTS_PER_ARM> enabled{};
    std::array<double, NUM_JOINTS_PER_ARM> gear_ratios{};
    std::array<int, NUM_JOINTS_PER_ARM> sign{};
    std::array<double, NUM_JOINTS_PER_ARM> rated_torque{};
    int can_channel = 0;
    int pdo_auto_report_ms = 0;
    int use_pdo = 0;
};

struct TorqueSensorParams {
    std::array<unsigned int,NUM_JOINTS_PER_ARM> ids{};
    std::array<int, NUM_JOINTS_PER_ARM> enabled{};
    int device_type; // 0: CANABLE, 1: USBCANII
    int can_channel = 0;
};

struct HandParams {
    int id = 0;
    int16_t hand_cmd[6];
};

struct ControlParams {
    struct PositionControl {
        double dt = 0.0;
        double T = 0.0;
        bool use_hand = false;
        bool use_torque_sensor = false;
        std::array<double, NUM_JOINTS_PER_ARM> command_degrees{};
        std::array<double, 6> cartesian_command{};
    };

    struct VelocityControl {
        double interpolation_dt = 0.0;
        int interpolation_T = 0;
        std::array<double, NUM_JOINTS_PER_ARM> kp{};
        std::array<double, NUM_JOINTS_PER_ARM> max_velocity{};
        double error_tolerance = 0.0;
        bool use_torque_sensor = false;
        std::array<double, NUM_JOINTS_PER_ARM> command_degrees{};
    };

    struct ForceControl {
        struct Admittance {
            double dt = 0.0;
            int T = 0;
            std::array<double, NUM_JOINTS_PER_ARM> stiffness{};
            std::array<double, NUM_JOINTS_PER_ARM> damping{};
            std::array<double, NUM_JOINTS_PER_ARM> inertia{};
            std::array<double, NUM_JOINTS_PER_ARM> deadzone{};
            std::array<double, NUM_JOINTS_PER_ARM> output_limits{};
            bool use_torque_sensor = false;
        };

        struct GravityCompensation {
            double dt = 0.0;
            int T = 0;
            std::array<double, 28> dynamics_parameters{}; // 7 * 4
            std::array<double, NUM_JOINTS_PER_ARM> stiffness{};
            std::array<double, NUM_JOINTS_PER_ARM> damping{};
            bool use_torque_sensor = false;
        };

        Admittance admittance[NUM_ARM];
        GravityCompensation gravity_compensation[NUM_ARM];
    };

    PositionControl position[NUM_ARM];
    VelocityControl velocity[NUM_ARM];
    ForceControl force;

    PositionControl head_position;
    VelocityControl head_velocity;
};

struct RobotConfig {
    size_t num_arms = NUM_ARM;
    size_t num_joints_per_arm = NUM_JOINTS_PER_ARM;
    size_t num_joints_head = 2;
    double pulse_to_degree = 0.0;
    double pulse_to_radian = 0.0;

    KinematicParams kinematics[NUM_ARM+1]; // left arm, right arm, and head
    DynamicParms dynamic[NUM_ARM]; // left and right arm
    MotorParams motors[NUM_ARM+1];// left and right arm and head
    TorqueSensorParams torque_sensors[NUM_ARM];
    HandParams hands[NUM_ARM];
    ControlParams control;
};

class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& filename);
    RobotConfig& getRobotConfig(){ return robot_config_; }

private:
    RobotConfig robot_config_;

    void loadFromFile(const std::string& filename);
    void parseKinematics(const YAML::Node& node);
    void parseHardware(const YAML::Node& node);
    void parseControl(const YAML::Node& node);
    void parseDynamics(const YAML::Node& node);
    template<typename T, size_t N>
    void copyYAMLToArray(const YAML::Node& yaml_node, std::array<T, N>& dest) {
        if (!yaml_node || yaml_node.size() > N) {
            throw std::runtime_error("Invalid array size in YAML or node is empty");
        }
        for (size_t i = 0; i < yaml_node.size(); ++i) {
            dest[i] = yaml_node[i].as<T>();
        }
    };

    template<typename T, size_t N>
    void copyYAMLToArrayOpt(const YAML::Node& yaml_node, std::array<T, N>& dest, bool optional = false) {
        if (!yaml_node && optional) return;
        if (!yaml_node || yaml_node.size() != N) {
            throw std::runtime_error("Invalid or missing array in YAML");
        }
        for (size_t i = 0; i < N; ++i) {
            dest[i] = yaml_node[i].as<T>();
        }
    };
};

#define MAX_TRAJECTORY_POINTS 12
struct PourTea
{
    int num_points;
    int operation_mode[2]; // 0: position control, 1: velocity control
    double dt[2][MAX_TRAJECTORY_POINTS];
    double T[2][MAX_TRAJECTORY_POINTS];
    double vel_T[2][MAX_TRAJECTORY_POINTS];
    bool use_hand;
    double pour_tea_target[NUM_ARM][MAX_TRAJECTORY_POINTS][NUM_JOINTS_PER_ARM];
    int16_t hand_angles[NUM_ARM][MAX_TRAJECTORY_POINTS][6];
};

class DemoParamLoader {
public:
    PourTea pour_tea;
    
    explicit DemoParamLoader(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        // 基础参数验证
        parsePourTeaParams(config["pour_tea"]);
    }

private:
    void parsePourTeaParams(const YAML::Node& node) {
        // Parse basic parameters
        pour_tea.use_hand = node["use_hand"].as<bool>();
        pour_tea.num_points = node["num_points"].as<double>();
        // Handle different arm names in YAML (left_arm_t vs left_arm)
        const char* yaml_arm_names[NUM_ARM] = {"left_arm", "right_arm"};
        for (int i = 0; i < NUM_ARM; ++i) {
            const YAML::Node& arm_node = node[yaml_arm_names[i]];
            pour_tea.operation_mode[i] = arm_node["control_mode"].as<int>();
            //Parse joint angles trajectory
            int trajectory_points = std::min(static_cast<int>(arm_node["joint_angles"].size()), 10);
            
            for (int point = 0; point < trajectory_points; ++point) {
                const YAML::Node& point_data = arm_node["joint_angles"][point];
                const YAML::Node& hand_data = arm_node["hand_angles"][point];
                pour_tea.dt[i][point] = arm_node["dt"][point].as<double>();
                pour_tea.T[i][point] = arm_node["T"][point].as<double>();
                pour_tea.vel_T[i][point] = arm_node["vel_T"][point].as<double>();
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j) {
                    pour_tea.pour_tea_target[i][point][j] = point_data[j].as<double>();
                }
                for (int j = 0; j < 6; ++j) {
                    pour_tea.hand_angles[i][point][j] = hand_data[j].as<int>();
                }
            }
            
    
        }
    }
};