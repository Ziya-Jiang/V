#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include "robot_arm.h"
#include "config_loader.h"
#include "kinematics.h"
#include "can_fd.h"

////////////////////////////////////////////////////////////// @brief Define pos control trajectory
int NUM_POINTS = 8;
void control_arm(RobotArms& arm, const Robot& robot_config, int arm_index) {
    std::cout << "Start position control for arm " << arm_index << std::endl;

    arm.move_line_offline(robot_config.control.position[arm_index].cartesian_command.data(),
                                robot_config.control.position[arm_index].dt,
                                robot_config.control.position[arm_index].T);
}

int main(int argc, char *argv[]) {
    // Load configuration from config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robots = config_loader.getRobots();

    // Initialize CAN communication and pdo callback dispatcher
    canfd_init(robots[0].motors.can_channel, 0);
    canfd_init(robots[1].motors.can_channel, 0);
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    RobotArms arms[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arms[i]) RobotArms(i, robots[i]);
        if (arms[i].initialize() != 0) return -1;
        if (arms[i].set_control_mode(pos_control) != 0) return -2;
    }

    // Create two threads to control each arm independently but in parallel
    std::thread t0(control_arm, std::ref(arms[0]), std::cref(robots[0]), 0);
    std::thread t1(control_arm, std::ref(arms[1]), std::cref(robots[1]), 1);
    t0.join();
    t1.join();


    canfd_close(robots[0].motors.can_channel, 0);
    canfd_close(robots[1].motors.can_channel, 0);
    return 0;
}