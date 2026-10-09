#include <thread>
#include "robot_arm.h"
#include "config_loader.h"
#include "kinematics.h"
#include "can_fd.h"

void control_arm(RobotArms& arm, const Robot& robot_config, int arm_index) {
    // 使用robot_config.control.position[arm_index].command_degrees 等
    arm.move_joint(robot_config.control.position[arm_index].command_degrees.data(),
                             robot_config.control.position[arm_index].dt,
                             robot_config.control.position[arm_index].T);
}

int main(int argc, char* argv[]) {
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robots = config_loader.getRobots();

    // 初始化CAN
    canfd_init(robots[0].motors.can_channel, 0);
    canfd_init(robots[1].motors.can_channel, 0);

    RobotArms arms[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arms[i]) RobotArms(i, robots[i]);
        if (arms[i].initialize() != 0) return -1;
        if (arms[i].set_control_mode(pos_control) != 0) return -2;
    }

    std::thread t0(control_arm, std::ref(arms[0]), std::cref(robots[0]), 0);
    std::thread t1(control_arm, std::ref(arms[1]), std::cref(robots[1]), 1);
    t0.join();
    t1.join();

    canfd_close(robots[0].motors.can_channel, 0);
    canfd_close(robots[1].motors.can_channel, 0);
    return 0;
}