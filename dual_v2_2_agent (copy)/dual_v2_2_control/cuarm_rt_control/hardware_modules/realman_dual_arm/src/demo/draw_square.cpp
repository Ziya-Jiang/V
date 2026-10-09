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

void control_arm(RobotArms& arm, const Robot& robot_config, TrajectoryList& trajectory_list, int arm_index) {
    for (int traj = 0; traj < trajectory_list.num_cartesian_points; ++traj) {
        arm.move_line_online(trajectory_list.cartesian_target[arm_index][traj],
                                 trajectory_list.dt[arm_index],
                                 trajectory_list.T[arm_index]);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main(int argc, char *argv[]) {
    // Load configuration from config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robots = config_loader.getRobots();

    TrajectoryLoader traj_loader(base_path + "/trajectory.yaml");
    TrajectoryList& trajectory_list = traj_loader.trajectory;

    canfd_init(robots[0].motors.can_channel, 0);
    canfd_init(robots[1].motors.can_channel, 0);

    RobotArms arms[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arms[i]) RobotArms(i, robots[i]);
        if (arms[i].initialize() != 0) return -1;
        if (arms[i].set_control_mode(pos_control) != 0) return -2;
    }
    
    // Create two threads to control each arm independently but in parallel
    std::thread t0(control_arm, std::ref(arms[0]), std::cref(robots[0]), std::ref(trajectory_list), 0);
    std::thread t1(control_arm, std::ref(arms[1]), std::cref(robots[1]), std::ref(trajectory_list), 1);

    // Wait for both threads to finish
    t0.join();
    t1.join();

    canfd_close(robots[0].motors.can_channel, 0);
    canfd_close(robots[1].motors.can_channel, 0);
    return 0;
}