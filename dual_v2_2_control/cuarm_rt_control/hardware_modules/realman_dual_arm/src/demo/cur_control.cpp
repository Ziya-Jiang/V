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

bool exit_requested = false;

// Function to check if a key is pressed (non-blocking)
int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

// Function to get the pressed key
char getch() {
    char ch;
    if (kbhit()) {
        ch = getchar();
        return ch;
    }
    return 0;
}

void signal_handler(int signal) {
    exit_requested = true;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);
    
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robots = config_loader.getRobots();
    FILE *fwrite_data = fopen("calibration_points_right.txt", "a");
    // 初始化CAN
    canfd_init(robots[0].motors.can_channel, 0);
    canfd_init(robots[1].motors.can_channel, 0);

    RobotArms arms[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i) {
        new (&arms[i]) RobotArms(i, robots[i]);
        if (arms[i].initialize() != 0) return -1;
        if (arms[i].set_control_mode(cur_control) != 0) return -2;
    }
    
    
    double dt = 0.1;
    double act_cur[NUM_ARM][NUM_JOINTS_PER_ARM] = {0};
    double act_mdhpos[NUM_ARM][NUM_JOINTS_PER_ARM] = {0};
    double act_vel[NUM_ARM][NUM_JOINTS_PER_ARM] = {0};
    
    while (!exit_requested) {
        auto clock_start = std::chrono::steady_clock::now();
        
        // Check for keyboard input
        char key = getch();
        if (key == 's' || key == 'S') {
            fprintf(fwrite_data, "[");
            for (int i = 0; i < NUM_ARM; ++i) {
                for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j) {
                    if (arms[i].motor_enable_list_[j] == 0)
                        continue;
                    fprintf(fwrite_data, "%f, ", act_mdhpos[i][j]);
                }
            }
            fprintf(fwrite_data, "],\n");
            fflush(fwrite_data); // Ensure data is written immediately
            std::cout << "Data saved to calibration_points.txt" << std::endl;
        }
        
        for (int i = 0; i < NUM_ARM; ++i) {
            for (int j = 0; j < NUM_JOINTS_PER_ARM; ++j) {
                if (arms[i].motor_enable_list_[j] == 0)
                    continue;
                arms[i].set_current(j, 0);
                act_cur[i][j] = arms[i].get_current(j);
                act_mdhpos[i][j] = arms[i].get_mdh_position(j);
                act_vel[i][j] = arms[i].get_velocity(j);
                std::cout << "arm: " << i << " joint: " << j
                         << " | act pos: " << act_mdhpos[i][j] 
                         << " | act vel: " << act_vel[i][j]
                         << " | act cur: " << act_cur[i][j] << std::endl;
            }
        }
        
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
        if (elapsed_us < dt * 1000000)
            std::this_thread::sleep_for(std::chrono::microseconds((long)(dt * 1000000 - elapsed_us)));
    }
    
    // Cleanup
    arms[0].set_control_mode(pos_control);
    arms[1].set_control_mode(pos_control);
    fclose(fwrite_data);
    canfd_close(robots[0].motors.can_channel, 0);
    canfd_close(robots[1].motors.can_channel, 0);
    
    return 0;
}