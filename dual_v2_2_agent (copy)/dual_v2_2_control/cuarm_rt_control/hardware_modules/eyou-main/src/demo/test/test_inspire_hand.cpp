#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "config_loader.h"
#include "eyou_joints.h"
#include "inspire_hand.h"

bool enable_hand[2] = {true, false};
bool exit_requested = false;
void signal_handler(int signal)
{
    exit_requested = true;
}

void test_force_calibration(InspireHand& inspire_hands){
    printf("Calibrate force for hand 0\n");
    inspire_hands.calibrateForce(0);

    printf("Calibrate force for hand 1\n");
    inspire_hands.calibrateForce(1);
}

void test_clear_error(InspireHand& inspire_hands){
    for (int i = 0; i < 2; i++)
    {
        if (!enable_hand[i]) continue;
        printf("Clear error for hand %d.\n", i);
        inspire_hands.clearError(i);
    }
    
    //Get Error
    for (size_t i = 0; i < 2; i++)
    {
        if (!enable_hand[i]) continue;
        inspire_hands.getError(i);
        usleep(2000);
    }
    inspire_hands.decodeError();

}

void test_set_angle(InspireHand& inspire_hands)
{
    while (!exit_requested)
    {
        //Wait for user input
        std::string user_input;
        std::cout << "Enter a interger from 0 to 1000: ";
        getline(std::cin, user_input);
        
        try{
            int16_t input = std::stoi(user_input);
            int16_t hand_angle[6] = {input, input, input, input, input, input};

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.setAngles(i, hand_angle);
                usleep(2000);
            }
            usleep(200000);

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.getAngles(i);
                usleep(2000);
            }

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.getForces(i);
                usleep(2000);
            }

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.getCurrent(i);
                usleep(2000);
            }

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.getError(i);
                usleep(2000);
            }
            // inspire_hands.decodeError();

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.getStatus(i);
                usleep(2000);
            }
            // inspire_hands.decodeStatus();

            for (int i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                for (int j = 0; j < 6; j++)
                {
                    std::cout << "inspire_hand " << i << ", Joint " << j
                            << ", \tangle: " << inspire_hands.joint_angle_[i][j]
                            << ", \tforce: " << inspire_hands.joint_force_[i][j]
                            << ", \tcurrent: " << inspire_hands.joint_current_[i][j]
                            << ", \terror: " << inspire_hands.joint_error_[i][j]
                            << ", \tstatus: " << inspire_hands.joint_status_[i][j]
                            << std::endl;
                }
            }
        }catch(const std::invalid_argument& e){
            break;
        }
    }
}

void test_force_and_current_limit_reading(InspireHand& inspire_hands){
    for (int i = 0; i < 2; i++)
    {   
        if (!enable_hand[i]) continue;
        printf("Get current limit for hand %d.\n", i);
        inspire_hands.getCurrentLimit(i);

        printf("Get force limit for hand %d.\n", i);
        inspire_hands.getForceLimit(i);
    }
    
    for (int i = 0; i < 2; i++)
    {
        if (!enable_hand[i]) continue;
        for (int j = 0; j < 6; j++)
        {
            std::cout << "inspire_hand " << i << ", Joint " << j
                    << ", \tforce_limit: " << inspire_hands.joint_force_limit_[i][j]
                    << ", \tcurrent_limit: " << inspire_hands.joint_current_limit_[i][j]
                    << std::endl;
        }
    }
}

void test_force_and_current_reading(InspireHand& inspire_hands, std::ofstream& fwriter)
{   
    int cnt = 0;
    int16_t hand_angle[][6] = {{1000, 1000, 1000, 1000, 1000, 1000}, {800, 800, 820, 800, 800, 0}};
    int next_action_idx = -1;
    int delay_ms = 50;
    while (!exit_requested)
    {
        std::cout <<"cnt: " << cnt << std::endl;
        if (cnt % 30 == 0){
            next_action_idx = (next_action_idx + 1) % 2;
            std::cout <<"Set action : " << next_action_idx <<  std::endl;
            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.setAngles(i, hand_angle[next_action_idx]);
                usleep(2000);
            }
        }

        for (size_t i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            inspire_hands.getAngles(i);
            usleep(2000);
        }

        for (size_t i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            inspire_hands.getForces(i);
            usleep(2000);
        }

        for (size_t i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            inspire_hands.getCurrent(i);
            usleep(2000);
        }

        for (int i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            for (int j = 0; j < 6; j++)
            {
                std::cout << "inspire_hand " << i << ", Joint " << j
                        << ", \tangle: " << inspire_hands.joint_angle_[i][j]
                        << ", \tforce: " << inspire_hands.joint_force_[i][j]
                        << ", \tcurrent: " << inspire_hands.joint_current_[i][j]
                        << std::endl;
            }
        }


        for (int i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            for (int j = 0; j < 6; j++)
            {
                fwriter << inspire_hands.joint_angle_[i][j] << " ";
            }
        }

        for (int i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            for (int j = 0; j < 6; j++)
            {
                fwriter << inspire_hands.joint_force_[i][j] << " ";
            }
        }

        for (int i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            for (int j = 0; j < 6; j++)
            {
                fwriter << inspire_hands.joint_current_[i][j] << " ";
            }
        }

        for (int i = 0; i < 2; i++)
        {
            if (!enable_hand[i]) continue;
            for (int j = 0; j < 6; j++)
            {
                fwriter << inspire_hands.joint_error_[i][j] << " ";
            }
        }

        fwriter << std::endl;
        cnt++;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

void test_set_angle_seperatedly(InspireHand& inspire_hands)
{
    while (!exit_requested)
    {
        //Wait for user input
        std::string user_input;
        std::cout << "Enter a list of interger from 0 to 1000 e.g.[1000 1000 1000 1000 1000 1000]: ";
        getline(std::cin, user_input);
        
        try{
            std::stringstream ss(user_input); // Create a stringstream from the input line
            int16_t hand_angle[6];
            int num;
            int cnt = 0;

            // Extract integers from the stringstream until no more integers can be read
            while (ss >> num && cnt < 6) {
                hand_angle[cnt] = static_cast<int16_t>(num);
                cnt++;
            }

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.setAngles(i, hand_angle);
                usleep(2000);
            }
            usleep(200000);

            for (size_t i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                inspire_hands.getAngles(i);
                usleep(2000);
            }

            // for (size_t i = 0; i < 2; i++)
            // {
            //     if (!enable_hand[i]) continue;
            //     inspire_hands.getForces(i);
            //     usleep(2000);
            // }

            // for (size_t i = 0; i < 2; i++)
            // {
            //     if (!enable_hand[i]) continue;
            //     inspire_hands.getCurrent(i);
            //     usleep(2000);
            // }

            // for (size_t i = 0; i < 2; i++)
            // {
            //     if (!enable_hand[i]) continue;
            //     inspire_hands.getError(i);
            //     usleep(2000);
            // }
            // // inspire_hands.decodeError();

            // for (size_t i = 0; i < 2; i++)
            // {
            //     if (!enable_hand[i]) continue;
            //     inspire_hands.getStatus(i);
            //     usleep(2000);
            // }
            // // inspire_hands.decodeStatus();

            for (int i = 0; i < 2; i++)
            {
                if (!enable_hand[i]) continue;
                for (int j = 0; j < 6; j++)
                {
                    std::cout << "inspire_hand " << i << ", Joint " << j
                            << ", \tangle: " << inspire_hands.joint_angle_[i][j]
                            // << ", \tforce: " << inspire_hands.joint_force_[i][j]
                            // << ", \tcurrent: " << inspire_hands.joint_current_[i][j]
                            // << ", \terror: " << inspire_hands.joint_error_[i][j]
                            // << ", \tstatus: " << inspire_hands.joint_status_[i][j]
                            << std::endl;
                }
            }
        }catch(const std::invalid_argument& e){
            break;
        }
    }
}

int main()
{
    signal(SIGINT, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path + "/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.motor.ids,
                        robot_config.hardware.motor.enabled);
    InspireHand inspire_hands(robot_config.hardware.hand.ids);
    std::ofstream fwriter("test_inspire_hand.txt");

    dual_arm.canable_init();

    int16_t gripper_speed[6] = {100, 100, 100, 100, 100, 100};
    int16_t gripper_force[6] = {300, 300, 300, 300, 300, 300};
    for (int i = 0; i < 2; i++)
    {   
        if (!enable_hand[i]) continue;
        inspire_hands.setSpeed(i, gripper_speed);
        inspire_hands.setForce(i, gripper_force);
    }

    std::cout <<"Start\n";
    test_set_angle(inspire_hands);
    // test_set_angle_seperatedly(inspire_hands);
    // test_force_and_current_reading(inspire_hands, fwriter);
    // test_force_and_current_limit_reading(inspire_hands);
    // test_force_calibration(inspire_hands);
    // test_clear_error(inspire_hands);

    dual_arm.canable_close();
    fwriter.close();
    return 0;
}