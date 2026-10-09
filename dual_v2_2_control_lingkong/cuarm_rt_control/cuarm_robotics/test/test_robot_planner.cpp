#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include "robot_planner.h"
#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace {

void initializeRobotState(RobotState& robot_state, const YAML::Node& config)
{
    robot_state.ArmSize = static_cast<uint8_t>(config["robot"]["arm"].size());
    for (int arm_i = 0; arm_i < robot_state.ArmSize; ++arm_i) {
        robot_state.JointSize[arm_i] =
            config["robot"]["arm"][arm_i]["joint_size"].as<int>();
        for (int joint_i = 0; joint_i < robot_state.JointSize[arm_i]; ++joint_i) {
            robot_state.MotorPos[arm_i][joint_i] = 0.0f;
            robot_state.MotorVel[arm_i][joint_i] = 0.0f;
            robot_state.MotorTor[arm_i][joint_i] = 0.0f;
        }
    }

    if (config["robot"]["gripper"]) {
        robot_state.GripperSize = static_cast<uint8_t>(config["robot"]["gripper"].size());
        for (int gripper_i = 0; gripper_i < robot_state.GripperSize; ++gripper_i) {
            robot_state.GripperJointSize[gripper_i] =
                config["robot"]["gripper"][gripper_i]["joint_size"].as<int>();
            for (int joint_i = 0; joint_i < robot_state.GripperJointSize[gripper_i]; ++joint_i) {
                robot_state.GripperJointPos[gripper_i][joint_i] = 0.0f;
            }
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
    panel_command.arm_target_mode = PanelTargetMode::kSinglePoint;
    panel_command.target_type = ControlType::kPosition;
    panel_command.actuator_mode = ControlType::kPosition;
    panel_command.interpolation_type = InterpolationMethod::kCos;
    panel_command.interpolation_speed_ratio = 1.0f;
    panel_command.InterpolationAccTime = 5.0f;
    panel_command.InterpolationConstVelTime = 1.0f;
    panel_command.NoneInterpolationSaturationRatio = 0.15f;
    panel_command.control_algorithm = ControlAlgorithm::kNone;

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
            panel_command.JointCmd[arm_i][joint_i] = 0.0f;
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

void setJointTarget(PanelCommand& panel_command, int arm_i, int joint_i, float target_rad)
{
    panel_command.JointCmd[arm_i][joint_i] = target_rad;
    panel_command.JointCmdDeg[arm_i][joint_i] = target_rad * 180.0f / static_cast<float>(PI);
}

void simulateRobotState(RobotState& robot_state, const PlannerState& planner_state)
{
    for (int arm_i = 0; arm_i < robot_state.ArmSize; ++arm_i) {
        for (int joint_i = 0; joint_i < robot_state.JointSize[arm_i]; ++joint_i) {
            robot_state.MotorPos[arm_i][joint_i] = planner_state.MotorPos[arm_i][joint_i];
            robot_state.MotorVel[arm_i][joint_i] = planner_state.MotorVel[arm_i][joint_i];
            robot_state.MotorTor[arm_i][joint_i] = planner_state.MotorTor[arm_i][joint_i];
        }
    }
}

void printJointTargets(const PlannerCommand& planner_command)
{
    for (int arm_i = 0; arm_i < planner_command.ArmSize; ++arm_i) {
        std::cout << "Arm " << arm_i << " planner target [deg]: ";
        for (int joint_i = 0; joint_i < planner_command.JointSize[arm_i]; ++joint_i) {
            std::cout << planner_command.Target[arm_i][joint_i] * 180.0 / PI << " ";
        }
        std::cout << std::endl;
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    const std::string config_prefix_path = CONFIG_PREFIX_PATH;
    const std::filesystem::path config_path =
        std::filesystem::path(config_prefix_path) / "config.yaml";
    constexpr int kLoopCount = 100;
    
    try {
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "Config file not found: " << config_path.string() << std::endl;
            std::cerr << "Usage: " << argv[0] << " [config_prefix_path]" << std::endl;
            return 1;
        }

        std::cout << "Loading config: " << config_path.string() << std::endl;
        YAML::Node config = loadYamlConfig(config_path.string());

        RobotState robot_state{};
        PanelCommand panel_command{};
        initializeRobotState(robot_state, config);
        initializePanelCommand(panel_command, config);

        const float dt = config["rt_control"]["dt_send"].as<float>();
        RobotPlanner planner(config, config_prefix_path);

        PlannerCommand* planner_command_ptr = nullptr;
        PlannerState* planner_state_ptr = nullptr;
        planner.getPlannerCommandAndPlannerState(planner_command_ptr, planner_state_ptr);

        planner_state_ptr->system_state = SystemState::kStartup;
        planner_state_ptr->plan_result = PlanResult::kSuccess;
        planner_state_ptr->system_diagnostic_flags = DiagnosticFlags::kNone;
        for (int arm_i = 0; arm_i < planner_state_ptr->ArmSize; ++arm_i) {
            for (int joint_i = 0; joint_i < planner_state_ptr->JointSize[arm_i]; ++joint_i) {
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] = DiagnosticFlags::kNone;
            }
        }

        if (panel_command.ArmSize > 0 && panel_command.JointSize[0] > 0) {
            setJointTarget(panel_command, 0, 0, 10.0f * static_cast<float>(PI) / 180.0f);
        }

        std::cout << "Running planner simulation for " << kLoopCount
                  << " cycles (dt=" << dt << " s)..." << std::endl;

        for (int step = 0; step < kLoopCount; ++step) {
            panel_command.send_timestamp = step;
            panel_command.sequence_id = static_cast<uint32_t>(step);

            planner.update(panel_command, robot_state);
            simulateRobotState(robot_state, *planner_state_ptr);
            planner.updatePlannerState(robot_state);

            if (step == 0 || step == kLoopCount - 1 || step % 20 == 0) {
                std::cout << "\n--- Step " << step << " ---" << std::endl;
                std::cout << "System state: " << magic_enum::enum_name(planner_state_ptr->system_state)
                          << ", plan result: " << magic_enum::enum_name(planner_state_ptr->plan_result)
                          << std::endl;
                printJointTargets(*planner_command_ptr);
            }

            std::this_thread::sleep_for(std::chrono::duration<float>(dt));
        }

        std::cout << "\nFinal planner state:" << std::endl;
        print_planner_state(planner_state_ptr);
        std::cout << "Robot planner test finished successfully." << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
