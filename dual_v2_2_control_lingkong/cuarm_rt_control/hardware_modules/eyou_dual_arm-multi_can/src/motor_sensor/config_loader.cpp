#include "config_loader.h"

ConfigLoader::ConfigLoader(const std::string& filename) {
    loadFromFile(filename);
}
void ConfigLoader::loadFromFile(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);

    if (!config["robot"] || !config["kinematics"] || !config["hardware"] || !config["control"]) {
        throw std::runtime_error("Missing required sections in config file");
    }

    robot_config_.num_joints_per_arm = config["robot"]["num_joints_per_arm"].as<size_t>();
    robot_config_.num_joints_head = config["robot"]["num_joints_head"].as<size_t>();
    robot_config_.pulse_to_degree = config["robot"]["pulse_to_degree"].as<double>();
    robot_config_.pulse_to_radian = config["robot"]["pulse_to_radian"].as<double>();

    parseKinematics(config["kinematics"]);
    parseDynamics(config["dynamics"]);
    parseHardware(config["hardware"]);
    parseControl(config["control"]);
}

void ConfigLoader::parseKinematics(const YAML::Node& node) {
    const char* arm_names[NUM_ARM+1] = {"left_arm", "right_arm", "head"};
    
    for (int i = 0; i < (NUM_ARM+1); ++i) {
        auto& kin = robot_config_.kinematics[i];
        const YAML::Node& arm_node = node[arm_names[i]];

        if (i < NUM_ARM) {
            // Left and right arm: 7 joints
            copyYAMLToArray(arm_node["a"], kin.a);
            copyYAMLToArray(arm_node["alpha"], kin.alpha);
            copyYAMLToArray(arm_node["d"], kin.d);
            copyYAMLToArray(arm_node["sign"], kin.sign);
            copyYAMLToArray(arm_node["base_to_world"], kin.base_to_world);
            copyYAMLToArray(arm_node["offset"], kin.offset);
            copyYAMLToArray(arm_node["init_mdhpos"], kin.initial_position);

            // Update global robot array
            for (size_t j = 0; j < NUM_JOINTS_PER_ARM; ++j) {
                g_robot[i].m_mdh.a[j] = kin.a[j];
                g_robot[i].m_mdh.alpha[j] = kin.alpha[j];
                g_robot[i].m_mdh.d[j] = kin.d[j];
                g_robot[i].m_mdh.offset[j] = kin.offset[j];
                g_robot[i].m_mdh.sign[j] = robot_config_.motors[i].sign[j];
                g_robot[i].m_mdh.m_init_mdhpos[j] = kin.initial_position[j];
                g_robot[i].m_ratio.joint_ratio[j] = robot_config_.motors[i].gear_ratios[j];
            }
            for (size_t j = 0; j < 6; ++j) {
                g_robot[i].m_mdh.base_to_world[j] = kin.base_to_world[j];
            }
        } else {
            // Head: only 2 joints, copy only first NUM_JOINTS_HEAD elements
            if (arm_node["a"] && arm_node["a"].size() >= NUM_JOINTS_HEAD) {
                for (size_t j = 0; j < NUM_JOINTS_HEAD; ++j) {
                    kin.a[j] = arm_node["a"][j].as<double>();
                }
            }
            if (arm_node["alpha"] && arm_node["alpha"].size() >= NUM_JOINTS_HEAD) {
                for (size_t j = 0; j < NUM_JOINTS_HEAD; ++j) {
                    kin.alpha[j] = arm_node["alpha"][j].as<double>();
                }
            }
            if (arm_node["d"] && arm_node["d"].size() >= NUM_JOINTS_HEAD) {
                for (size_t j = 0; j < NUM_JOINTS_HEAD; ++j) {
                    kin.d[j] = arm_node["d"][j].as<double>();
                }
            }
            if (arm_node["sign"] && arm_node["sign"].size() >= NUM_JOINTS_HEAD) {
                for (size_t j = 0; j < NUM_JOINTS_HEAD; ++j) {
                    kin.sign[j] = arm_node["sign"][j].as<int>();
                }
            }
            copyYAMLToArray(arm_node["base_to_world"], kin.base_to_world);
            if (arm_node["offset"] && arm_node["offset"].size() >= NUM_JOINTS_HEAD) {
                for (size_t j = 0; j < NUM_JOINTS_HEAD; ++j) {
                    kin.offset[j] = arm_node["offset"][j].as<double>();
                }
            }
            if (arm_node["init_mdhpos"] && arm_node["init_mdhpos"].size() >= NUM_JOINTS_HEAD) {
                for (size_t j = 0; j < NUM_JOINTS_HEAD; ++j) {
                    kin.initial_position[j] = arm_node["init_mdhpos"][j].as<double>();
                }
            }
        }
    }
}
void ConfigLoader::parseDynamics(const YAML::Node& node) {
    const char* arm_names[NUM_ARM] = {"left_arm", "right_arm"};
    for (int i = 0; i < NUM_ARM; ++i) {
        auto& dyn = robot_config_.dynamic[i];
        const YAML::Node& arm_node = node[arm_names[i]];

        copyYAMLToArray(arm_node["dynamics_parm"], dyn.dynamic_parameters);
    }
}
void ConfigLoader::parseHardware(const YAML::Node& node) {
    const char* arm_names[NUM_ARM+1] = {"left_arm", "right_arm", "head"};

    for (int i = 0; i < (NUM_ARM+1); ++i) {
        auto& motor = robot_config_.motors[i];
        const YAML::Node& motor_node = node["motor"][arm_names[i]];

        copyYAMLToArray(motor_node["ids"], motor.ids);
        copyYAMLToArray(motor_node["enabled"], motor.enabled);
        copyYAMLToArray(motor_node["gear_ratios"], motor.gear_ratios);
        copyYAMLToArray(motor_node["sign"], motor.sign);
        copyYAMLToArray(motor_node["rated_torque"], motor.rated_torque);
        motor.use_pdo = node["can"]["use_pdo"].as<bool>();
        motor.can_channel = node["can"][arm_names[i]]["channel"].as<int>();
        motor.pdo_auto_report_ms = node["can"][arm_names[i]]["pdo_auto_report_ms"].as<int>();
    }

    for (int i = 0; i < NUM_ARM; ++i) {
        auto& ts = robot_config_.torque_sensors[i];
        const YAML::Node& ts_node = node["torque_sensor"][arm_names[i]];

        copyYAMLToArray(ts_node["ids"], ts.ids);
        copyYAMLToArray(ts_node["enabled"], ts.enabled);
        ts.can_channel = node["can"]["torque_sensor"]["channel"][i].as<int>();
        ts.device_type = node["can"]["torque_sensor"]["device_type"].as<int>();
    }

    for (int i = 0; i < NUM_ARM; ++i) {
        robot_config_.hands[i].id = node["hand"][arm_names[i]]["id"].as<int>();
        for (int j = 0;j<6;++j)
        {
            robot_config_.hands[i].hand_cmd[j]= node["hand"][arm_names[i]]["cmd"][j].as<int16_t>();
        }
        
        
    }

}

void ConfigLoader::parseControl(const YAML::Node& node) {
    const char* arm_names[NUM_ARM] = {"left_arm", "right_arm"};
    const YAML::Node& pos_node = node["position"];
    const YAML::Node& vel_node = node["velocity"];
    const YAML::Node& force_node = node["force"];

    // Position control
    for (int i = 0; i < NUM_ARM; ++i) {
        auto& pc = robot_config_.control.position[i];
        const YAML::Node& arm_node = pos_node[arm_names[i]];

        pc.dt = arm_node["dt"].as<double>();
        pc.T = arm_node["T"].as<double>();
        pc.use_torque_sensor = arm_node["use_torque_sensor"].as<bool>();
        copyYAMLToArray(arm_node["command_degrees"], pc.command_degrees);
        copyYAMLToArray(arm_node["cartesian_command"], pc.cartesian_command);
    }

    // Velocity control
    for (int i = 0; i < NUM_ARM; ++i) {
        auto& vc = robot_config_.control.velocity[i];
        const YAML::Node& arm_node = vel_node[arm_names[i]];

        vc.interpolation_dt = arm_node["interpolation_dt"].as<double>();
        vc.interpolation_T = arm_node["interpolation_T"].as<int>();
        vc.error_tolerance = arm_node["error_tolerance"].as<double>();
        vc.use_torque_sensor = arm_node["use_torque_sensor"].as<bool>();
        copyYAMLToArray(arm_node["kp"], vc.kp);
        copyYAMLToArray(arm_node["max_velocity"], vc.max_velocity);
        copyYAMLToArray(arm_node["command_degrees"], vc.command_degrees);
    }

    // Force control
    const YAML::Node& ac_node = force_node["admittance"];
    const YAML::Node& gc_node = force_node["gravity_compensation"];

    for (int i = 0; i < NUM_ARM; ++i) {
        auto& ac = robot_config_.control.force.admittance[i];
        const YAML::Node& arm_node = ac_node[arm_names[i]];

        ac.dt = arm_node["dt"].as<double>();
        ac.T = arm_node["T"].as<int>();
        copyYAMLToArray(arm_node["K"], ac.stiffness);
        copyYAMLToArray(arm_node["damp"], ac.damping);
        copyYAMLToArray(arm_node["inertia"], ac.inertia);
        copyYAMLToArray(arm_node["deadzone"], ac.deadzone);
        copyYAMLToArray(arm_node["output_limits"], ac.output_limits);
    }

    for (int i = 0; i < NUM_ARM; ++i) {
        auto& gc = robot_config_.control.force.gravity_compensation[i];
        const YAML::Node& arm_node = gc_node[arm_names[i]];

        gc.dt = arm_node["dt"].as<double>();
        gc.T = arm_node["T"].as<int>();
        gc.use_torque_sensor = arm_node["use_torque_sensor"].as<bool>();
        copyYAMLToArray(arm_node["K"], gc.stiffness);
        copyYAMLToArray(arm_node["damp"], gc.damping);
    }
    //___________________________________________________________________________
    // Modified by WG: add head position control
    const YAML::Node& head_node = pos_node["head"];
    if (head_node) {
        auto& hpc = robot_config_.control.head_position;
        hpc.dt = head_node["dt"].as<double>();
        hpc.T = head_node["T"].as<double>();
        if (head_node["use_torque_sensor"]) {
            hpc.use_torque_sensor = head_node["use_torque_sensor"].as<bool>();
        }
        if (head_node["use_hand"]) {
            hpc.use_hand = head_node["use_hand"].as<bool>();
        }
        if (head_node["command_degrees"]) {
            // Head 只有 2 个关节，需要特殊处理
            std::array<double, NUM_JOINTS_HEAD> head_cmd_degrees{};
            copyYAMLToArray(head_node["command_degrees"], head_cmd_degrees);
            // 只复制前 NUM_JOINTS_HEAD 个值到 command_degrees 数组
            for (size_t i = 0; i < NUM_JOINTS_HEAD; ++i) {
                hpc.command_degrees[i] = head_cmd_degrees[i];
            }
        }
    }
}