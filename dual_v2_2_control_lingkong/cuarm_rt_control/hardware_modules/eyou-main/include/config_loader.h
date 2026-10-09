#pragma once
#include <yaml-cpp/yaml.h>
#include <array>
#include <cstdint>
#include <stdexcept>
#include "kinematics.h"
const size_t NUM_JOINTS = 14; // 硬编码为14，与YAML中的num_joints对应

struct HardwareParams {
    struct Motor {
        uint32_t ids[NUM_JOINTS];
        int enabled[NUM_JOINTS];
        int gear_ratios[NUM_JOINTS];
        int sign[NUM_JOINTS];
        int32_t init_pos_pulse[NUM_JOINTS];
        double init_pos_degrees[NUM_JOINTS];
    };
    struct Head_Motor {
        uint32_t ids[NUM_JOINTS];
        int enabled[NUM_JOINTS];
        int gear_ratios[NUM_JOINTS];
        int sign[NUM_JOINTS];
    };
    struct Hand {
       int ids[2]; // 固定2个手爪
    };

    struct TorqueSensor {
        uint32_t ids[NUM_JOINTS];
        int enabled[NUM_JOINTS];
    };

    Motor motor;
    Hand hand;
    Head_Motor head_motor;
    TorqueSensor torque_sensor;
    bool use_camera;
};

struct MDHParams {
    double left_a[7];
    double left_alpha[7];
    double left_d[7];
    int left_sign[7];   
    double left_base_to_world[6];
    double left_offset[7];
    double left_init_mdhpos[7];

    double right_a[7];
    double right_alpha[7];
    double right_d[7];
    int right_sign[7];
    double right_base_to_world[6];
    double right_offset[7];
    double right_init_mdhpos[7];
};
struct PositionControlParams {
    double dt;
    double T;
    bool use_tor;
    bool use_hand;
    double cmd[NUM_JOINTS];
};
struct VelocityControlParams {
    double dt;
    double Kp[NUM_JOINTS];
    double max_velocity[NUM_JOINTS];
    int T;
    bool use_tor;
    bool use_hand;
    double error_tol;
    double cmd[NUM_JOINTS];
};
struct ForceControlParams {
    double dt;
    double T;
    struct Admittance {
        double K[NUM_JOINTS];
        double damp[NUM_JOINTS];
        double inertia[NUM_JOINTS];
        double deadzone[NUM_JOINTS];
        double output_limits[NUM_JOINTS];
        bool use_tor;
    };
    struct Gravity_Compensation {
        double left_arm_dynamics[NUM_JOINTS/2*4];
        double right_arm_dynamics[NUM_JOINTS/2*4];
        double K[NUM_JOINTS];
        double damp[NUM_JOINTS];
        bool use_tor;
    };

    Admittance admittance;
    Gravity_Compensation gravity_compensation;
};

class ConfigLoader {
public:
    HardwareParams hardware;
    PositionControlParams position_control;
    ForceControlParams force_control;
    MDHParams mdh_parm;
    VelocityControlParams velocity_control;
    explicit ConfigLoader(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);

        // 基础参数验证
        parseMDHParams(config["mdh"]);
        parseHardware(config["hardware"]);
        parsePositionControl(config["position_control"]);
        parseForceControl(config["force_control"]);
        parseVelocityControl(config["velocity_control"]);
    }

private:
    
    void parseMDHParams(const YAML::Node& node)
    {
        for (int i = 0;i<NUM_JOINTS/2; ++i)
        {
            mdh_parm.left_a[i] = node["left_a"][i].as<double>();
            mdh_parm.left_alpha[i] = node["left_alpha"][i].as<double>();
            mdh_parm.left_d[i] = node["left_d"][i].as<double>();
            mdh_parm.left_sign[i] = node["left_sign"][i].as<int>();
            mdh_parm.left_offset[i] = node["left_offset"][i].as<double>();
            mdh_parm.left_init_mdhpos[i] = node["left_init_mdhpos"][i].as<double>();

            mdh_parm.right_a[i] = node["right_a"][i].as<double>();
            mdh_parm.right_alpha[i] = node["right_alpha"][i].as<double>();
            mdh_parm.right_d[i] = node["right_d"][i].as<double>();
            mdh_parm.right_sign[i] = node["right_sign"][i].as<int>();
            mdh_parm.right_offset[i] = node["right_offset"][i].as<double>();
            mdh_parm.right_init_mdhpos[i] = node["right_init_mdhpos"][i].as<double>();

            g_robot[0].m_mdh.a[i] = mdh_parm.left_a[i];
            g_robot[0].m_mdh.alpha[i] = mdh_parm.left_alpha[i];
            g_robot[0].m_mdh.d[i] = mdh_parm.left_d[i];
            g_robot[0].m_mdh.offset[i] = mdh_parm.left_offset[i];
            g_robot[0].m_mdh.sign[i] = mdh_parm.left_sign[i];
            g_robot[0].m_mdh.m_init_mdhpos[i] = mdh_parm.left_init_mdhpos[i];
            g_robot[1].m_mdh.a[i] = mdh_parm.right_a[i];
            g_robot[1].m_mdh.alpha[i] = mdh_parm.right_alpha[i];
            g_robot[1].m_mdh.d[i] = mdh_parm.right_d[i];
            g_robot[1].m_mdh.offset[i] = mdh_parm.right_offset[i];
            g_robot[1].m_mdh.sign[i] = mdh_parm.right_sign[i];
            g_robot[1].m_mdh.m_init_mdhpos[i] = mdh_parm.right_init_mdhpos[i];

        }
        for (int i = 0;i<6;++i)
        {
            mdh_parm.left_base_to_world[i] = node["left_base_to_world"][i].as<double>();
            mdh_parm.right_base_to_world[i] = node["right_base_to_world"][i].as<double>();
            g_robot[0].m_mdh.base_to_world[i] = mdh_parm.left_base_to_world[i];
            g_robot[1].m_mdh.base_to_world[i] = mdh_parm.right_base_to_world[i];
        }
    }

    void parseHardware(const YAML::Node& node) {
        // 解析电机参数
        hardware.use_camera = node["camera"]["use_camera"].as<bool>();
        for (int i = 0;i<NUM_JOINTS;++i)
        {
            hardware.motor.ids[i] = node["motor"]["ids"][i].as<uint32_t>();
            hardware.motor.enabled[i] = node["motor"]["enabled"][i].as<int>();
            hardware.motor.gear_ratios[i] = node["motor"]["gear_ratios"][i].as<int>();
            hardware.motor.sign[i] = node["motor"]["sign"][i].as<int>();
            hardware.motor.init_pos_pulse[i] = node["motor"]["init_pos_pulse"][i].as<int32_t>();
            hardware.motor.init_pos_degrees[i] = node["motor"]["init_pos_degrees"][i].as<double>();

            hardware.head_motor.sign[i] = node["head_motor"]["sign"][i].as<int>();
            hardware.head_motor.gear_ratios[i] = node["head_motor"]["gear_ratios"][i].as<int>();
            hardware.head_motor.ids[i] = node["head_motor"]["ids"][i].as<uint32_t>();
            hardware.head_motor.enabled[i] = node["head_motor"]["enabled"][i].as<int>();

            hardware.torque_sensor.ids[i] = node["torque_sensor"]["ids"][i].as<uint32_t>();
            hardware.torque_sensor.enabled[i] = node["torque_sensor"]["enabled"][i].as<int>();
        }

        // 解析手部电机参数
        for(int i = 0;i<2;++i)
        {
            hardware.hand.ids[i] = node["hand"]["ids"][i].as<int>();
        }
        // 解析手爪ID（处理16进制）
    }

    void parsePositionControl(const YAML::Node& node) {
        position_control.dt = node["dt"].as<double>();
        position_control.T = node["T"].as<double>();
        position_control.use_hand= node["hand"].as<bool>();
        position_control.use_tor= node["torque_sensor"].as<bool>();
        for (int i = 0; i < NUM_JOINTS; ++i) {
            position_control.cmd[i] = node["cmd"][i].as<double>();
        }
    }
    void parseVelocityControl(const YAML::Node& node) {
        velocity_control.dt = node["dt"].as<double>();
        velocity_control.use_hand= node["hand"].as<bool>();
        velocity_control.use_tor= node["torque_sensor"].as<bool>();
        velocity_control.error_tol = node["error_tol"].as<double>();
        velocity_control.T = node["T"].as<int>();
        for (int i = 0; i < NUM_JOINTS; ++i) {
            velocity_control.Kp[i] = node["Kp"][i].as<double>();
            velocity_control.max_velocity[i] = node["max_velocity"][i].as<double>();
            velocity_control.cmd[i] = node["cmd"][i].as<double>();
        }
    }

    void parseForceControl(const YAML::Node& node) {
        force_control.dt = node["dt"].as<double>();
        force_control.T = node["T"].as<int>();
        force_control.admittance.use_tor = node["admittance"]["torque_sensor"].as<bool>();
        force_control.gravity_compensation.use_tor = node["gravity_compensation"]["torque_sensor"].as<bool>();

        for (int i = 0; i < NUM_JOINTS; ++i) {
            force_control.admittance.K[i] = node["admittance"]["K"][i].as<double>();
            force_control.admittance.damp[i] = node["admittance"]["damp"][i].as<double>();
            force_control.admittance.inertia[i] = node["admittance"]["inertia"][i].as<double>();
            force_control.admittance.deadzone[i] = node["admittance"]["deadzone"][i].as<double>();
            force_control.admittance.output_limits[i] = node["admittance"]["output_limits"][i].as<double>();
        }
        
        for (int i = 0; i < NUM_JOINTS / 2*4; ++i) {
            force_control.gravity_compensation.left_arm_dynamics[i] = node["gravity_compensation"]["left_arm_dynamics"][i].as<double>();
            force_control.gravity_compensation.right_arm_dynamics[i] = node["gravity_compensation"]["right_arm_dynamics"][i].as<double>();
        }
        for (int i = 0; i < NUM_JOINTS; ++i) {
            force_control.gravity_compensation.K[i] = node["gravity_compensation"]["K"][i].as<double>();
            force_control.gravity_compensation.damp[i] = node["gravity_compensation"]["damp"][i].as<double>();
        }
    }

};
