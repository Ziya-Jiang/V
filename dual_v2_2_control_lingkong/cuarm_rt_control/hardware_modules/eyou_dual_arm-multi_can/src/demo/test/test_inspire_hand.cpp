#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include "eyou_joints.h"
#include "polynomial_trajectory.h"
#include "inspire_hand.h"
#include <unistd.h>
#include "config_loader.h"
#include "usb_to_can.h"
#include "callback_dispatcher.h"
// 主函数
bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}

int main()
{
    //Load configuration form config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();
    // Initialize CAN communication and pdo callback dispatcher
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    
    int hand_ids[2] = {robot_config.hands[0].id, robot_config.hands[1].id};
    int hand_channel[2] = {robot_config.motors[0].can_channel, robot_config.motors[1].can_channel};
    InspireHand inspire_hands(hand_ids, hand_channel);
    //-----------------------------------------------------
    int16_t force[6] = {50, 50, 50, 50, 50, 50};
    int16_t speed[6] = {200, 200, 200, 200, 200, 200};
    inspire_hands.setForce(0, force);
    inspire_hands.setAngles(0, robot_config.hands[0].hand_cmd);
    for (size_t i = 0; i < 2; i++)
    {
        inspire_hands.setAngles(0, robot_config.hands[0].hand_cmd);
        usleep(20000);
    }
    usleep(1000000);

    // inspire_hands.setForce
    int16_t ang[6] = {1000, 1000, 1000, 1000, 1000, 1000};
    while(!exit_requested)
    {
        for (int i = 0;i<6;++i)
        {
            if (ang[i] > 500)
            {
                ang[i]--;
            }  
        }
        //inspire_hands.setAngles(0, ang);
        inspire_hands.getForce(0);
        //inspire_hands.getAngles(0);
        //inspire_hands.getCurrent(0);
        //inspire_hands.getStatus(0);
        // inspire_hands.getStatus(0);
        for (int i = 0;i<6;++i)
        {
             std::cout<<" joint " << i <<" | force act" << inspire_hands.joint_force_[0][i]<<" | angle :"<< inspire_hands.joint_angle_[0][i]<<std::endl;
            //         <<" | current :"<< inspire_hands.joint_current_[0][i] 
            //         <<" | error :"<< inspire_hands.joint_error_[0][i] 
            //         <<" | status :"<< inspire_hands.joint_status_[0][i] <<std::endl;
        }
        std::cout <<std::endl;
        usleep(10000);
    }
    // for (size_t i = 0; i < 2; i++)
    // {
    //     inspire_hands.setAngles(i, robot_config.hands[i].hand_cmd);
    //     usleep(20000);
    // }
    // usleep(1000000);

    // for (size_t i = 0; i < 2; i++)
    // {
    //     inspire_hands.getAngles(i);
    // }
    // for (size_t i = 0; i < 2; i++)
    // {
    //     inspire_hands.setAngles(i, hand_angle1);
    //     usleep(20000);
    // }
    
    // printf("Right hand zero values: {%d %d %d %d %d %d}\n", zero_angle_right[0], zero_angle_right[1], zero_angle_right[2], 
    //                                                 zero_angle_right[3], zero_angle_right[4],  zero_angle_right[5]);
    // printf("left hand zero values: {%d %d %d %d %d %d}\n", zero_angle_left[0], zero_angle_left[1], zero_angle_left[2], 
    //                                                 zero_angle_left[3], zero_angle_left[4],  zero_angle_left[5]);                                                
    canbox_close({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}