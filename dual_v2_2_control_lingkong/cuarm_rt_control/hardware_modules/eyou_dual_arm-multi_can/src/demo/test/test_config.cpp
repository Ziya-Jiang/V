#include <iostream>
#include "config_loader.h"

int main() {
    try {
        // Load configuration from file
        std::string base_path = PROJECT_SOURCE_DIR;
        ConfigLoader loader(base_path+"/config.yaml");
        const RobotConfig& config = loader.getRobotConfig();

        std::cout << "Robot Configuration Loaded Successfully!\n\n";

        // Print left arm kinematic parameters
        std::cout << "Left Arm Kinematic Parameters:\n";
        std::cout << "a: ";
        for (const auto& val : config.kinematics[0].a) std::cout << val << " ";
        std::cout << "\nalpha: ";
        for (const auto& val : config.kinematics[0].alpha) std::cout << val << " ";
        std::cout << "\nd: ";
        for (const auto& val : config.kinematics[0].d) std::cout << val << " ";
        std::cout << "\nsign: ";
        for (const auto& val : config.kinematics[0].sign) std::cout << val << " ";
        std::cout << "\nbase_to_world: ";
        for (const auto& val : config.kinematics[0].base_to_world) std::cout << val << " ";
        std::cout << "\noffset: ";
        for (const auto& val : config.kinematics[0].offset) std::cout << val << " ";
        std::cout << "\ninitial_position: ";
        for (const auto& val : config.kinematics[0].initial_position) std::cout << val << " ";

        std::cout << "\n\nMotor IDs for Left Arm: ";
        for (const auto& id : config.motors[0].ids) std::cout << id << " ";

        std::cout << "\n\nPosition Control Command Degrees (Left Arm): ";
        for (const auto& deg : config.control.position[0].command_degrees) std::cout << deg << " ";
        std::cout << std::endl;
        std::cout << "\n\ntorque sensor enable (right Arm): ";
        for (const auto& deg : config.torque_sensors[1].enabled) std::cout << deg << " ";
        std::cout << std::endl;
        std::cout << "\n\ntorque sensor enable (left Arm): ";
        for (const auto& deg : config.torque_sensors[0].enabled) std::cout << deg << " ";
        std::cout << std::endl;
        std::cout << "Right Arm Kinematic Parameters:\n";
        std::cout << "a: ";
        for (const auto& val : config.kinematics[1].a) std::cout << val << " ";
        std::cout << "\nalpha: ";
        for (const auto& val : config.kinematics[1].alpha) std::cout << val << " ";
        std::cout << "\nd: ";
        for (const auto& val : config.kinematics[1].d) std::cout << val << " ";
        std::cout << "\nsign: ";
        for (const auto& val : config.kinematics[1].sign) std::cout << val << " ";
        std::cout << "\nbase_to_world: ";
        for (const auto& val : config.kinematics[1].base_to_world) std::cout << val << " ";
        std::cout << "\noffset: ";
        for (const auto& val : config.kinematics[1].offset) std::cout << val << " ";
        std::cout << "\ninitial_position: ";
        for (const auto& val : config.kinematics[1].initial_position) std::cout << val << " ";
        std::cout << "\n\nMotor IDs for Left Arm: ";
        for (const auto& id : config.motors[0].ids) std::cout << id << " ";
        std::cout << "\n\nMotor IDs for Right Arm: ";
        for (const auto& id : config.motors[1].ids) std::cout << id << " ";
        std::cout << "\n\nMotor IDs for Head: ";
        for (const auto& id : config.motors[2].ids) std::cout << id << " ";

        std::cout << "\n\nPosition Control Command Degrees (Right Arm): ";
        for (const auto& deg : config.control.position[1].command_degrees) std::cout << deg << " ";
        std::cout << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "Error loading config: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}