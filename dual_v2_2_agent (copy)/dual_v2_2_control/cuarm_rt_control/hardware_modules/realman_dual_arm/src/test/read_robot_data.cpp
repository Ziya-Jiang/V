#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "robot_arm.h"
#include "config_loader.h"
#include "kinematics.h"
#include "can_fd.h"

// Function to check if a key is pressed (non-blocking)

int main(int argc, char *argv[]) {
    // Load configuration from config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robots = config_loader.getRobots();
    TrajectoryLoader traj_loader(base_path + "/trajectory.yaml");
    TrajectoryList& trajectory_list = traj_loader.trajectory;
    // 初始化CAN
    canfd_init(robots[0].motors.can_channel, 0);
    canfd_init(robots[1].motors.can_channel, 0);

    RobotArms arms[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arms[i]) RobotArms(i, robots[i]);
        if (arms[i].initialize() != 0) return -1;
        if (arms[i].set_control_mode(pos_control) != 0) return -2;
    }

    // Cleanup
    canfd_close(robots[0].motors.can_channel, 0);
    canfd_close(robots[1].motors.can_channel, 0);
    return 0;
}