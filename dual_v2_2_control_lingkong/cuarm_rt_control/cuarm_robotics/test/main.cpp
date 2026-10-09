#include <cstring>
#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_planner.h"
#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"

namespace {

void initializeRobotState(RobotState& robot_state, const YAML::Node& config)
{
    robot_state.ArmSize = static_cast<uint8_t>(config["robot"]["arm"].size());
    for (int arm_i = 0; arm_i < robot_state.ArmSize; ++arm_i) {
        robot_state.JointSize[arm_i] =
            config["robot"]["arm"][arm_i]["joint_size"].as<int>();
    }

    if (config["robot"]["gripper"]) {
        robot_state.GripperSize = static_cast<uint8_t>(config["robot"]["gripper"].size());
        for (int gripper_i = 0; gripper_i < robot_state.GripperSize; ++gripper_i) {
            robot_state.GripperJointSize[gripper_i] =
                config["robot"]["gripper"][gripper_i]["joint_size"].as<int>();
        }
    } else {
        robot_state.GripperSize = 0;
    }
}

void initializePanelCommand(PanelCommand& panel_command, const YAML::Node& config)
{
    std::memset(&panel_command, 0, sizeof(PanelCommand));

    panel_command.connection_state = ConnectionState::kRemote;
    panel_command.simulation = true;
    panel_command.motion_type = MotionControl::kJoint;
    panel_command.target_type = ControlType::kPosition;
    panel_command.actuator_mode = ControlType::kPosition;

    panel_command.ArmSize = static_cast<uint8_t>(config["robot"]["arm"].size());
    for (int arm_i = 0; arm_i < panel_command.ArmSize; ++arm_i) {
        const std::string arm_name = config["robot"]["arm"][arm_i]["name"].as<std::string>();
        const std::string arm_type = config["robot"]["arm"][arm_i]["type"].as<std::string>();
        const std::string robot_name = arm_name + "-" + arm_type;

        panel_command.JointSize[arm_i] =
            config["robot"]["arm"][arm_i]["joint_size"].as<int>();
        std::strncpy(panel_command.RobotName[arm_i], robot_name.c_str(), MAX_ROBOTNAME_SIZE - 1);
        panel_command.RobotName[arm_i][MAX_ROBOTNAME_SIZE - 1] = '\0';

        for (int joint_i = 0; joint_i < panel_command.JointSize[arm_i]; ++joint_i) {
            panel_command.enable_joint[arm_i][joint_i] =
                config["robot"]["arm"][arm_i]["enable_joint"][joint_i].as<bool>();
        }
    }

    if (config["robot"]["gripper"]) {
        panel_command.GripperSize = static_cast<uint8_t>(config["robot"]["gripper"].size());
        for (int gripper_i = 0; gripper_i < panel_command.GripperSize; ++gripper_i) {
            panel_command.GripperJointSize[gripper_i] =
                config["robot"]["gripper"][gripper_i]["joint_size"].as<int>();
        }
    } else {
        panel_command.GripperSize = 0;
    }
}

}  // namespace

int main(int argc, char** argv)
{
    try {
#ifdef CONFIG_PREFIX_PATH
        std::string config_prefix_path = CONFIG_PREFIX_PATH;
#else
        std::string config_prefix_path = "../cuarm_configuration/arm_v1";
#endif
        if (argc > 1) {
            config_prefix_path = argv[1];
        }

        const std::string config_path = config_prefix_path + "/config.yaml";
        std::cout << "Loading config: " << config_path << std::endl;

        YAML::Node config = loadYamlConfig(config_path);

        RobotState robot_state{};
        PanelCommand panel_command{};
        initializeRobotState(robot_state, config);
        initializePanelCommand(panel_command, config);

        RobotPlanner planner(config, config_prefix_path);

        PlannerCommand* planner_command_ptr = nullptr;
        PlannerState* planner_state_ptr = nullptr;
        planner.getPlannerCommandAndPlannerState(planner_command_ptr, planner_state_ptr);

        constexpr int kLoopCount = 10;
        for (int step = 0; step < kLoopCount; ++step) {
            planner.update(panel_command, robot_state);
            planner.updatePlannerState(robot_state);
        }

        std::cout << "update() and updatePlannerState() ran " << kLoopCount
                  << " cycles without error." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
