#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <signal.h>
#include <fstream>

#include "inspire_hand.h"
#include "eyou_joints.h"
#include "config_loader.h"

std::string base_path = PROJECT_SOURCE_DIR;
std::string config_path = base_path+"/config.yaml";
ConfigLoader robot_config(config_path);
EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
InspireHand hands(robot_config.hardware.hand.ids);

void sigint_handler(int s)
{
    printf("Caught signal %d\n", s);
    // shutdown_all = true;
    // exit(0);
}
int main(int argc, char *argv[])
{
    signal(SIGINT, sigint_handler);

    // ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);

    dual_arm.all_motor_stop();
    dual_arm.canable_close();
    return 0;
}
