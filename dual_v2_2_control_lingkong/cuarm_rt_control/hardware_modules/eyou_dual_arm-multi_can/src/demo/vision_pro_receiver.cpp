#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include <map> 

#include "inspire_hand.h"
#include "eyou_joints.h"
#include "config_loader.h"
#include "callback_dispatcher.h"
#include "usb_to_can.h"

#include "cuarm_udp.h"
#include "cuarm_state.h"

#define NUM_JOINTS 14

bool use_curobo = false;
bool use_gripper_mode = true;
bool shutdown_all = false;

FILE *fwrite_data = fopen("data.txt", "w");

//Udp related
udp_node udp;
std::mutex receive_udp_mtx;
teleop_command received_command;
teleop_command teleop_command_;
teleop_robot_state teleop_robot_state_;

//Arm related
std::string base_path = PROJECT_SOURCE_DIR;
ConfigLoader config_loader(base_path + "/config.yaml");
RobotConfig& robot_config = config_loader.getRobotConfig();
float pos_limits[2][14] = {
    {-1.57f, 0.8f, -1.57f, 0.8f, -3.14f, -1.57f, -1.57f, -1.57f, 0.8f, -1.57f, 0.8f, -3.14f, -1.57f, -1.57f},
    {1.57f,  1.5f,  1.57f, 1.5f,  3.14f,  1.57f,  1.57f, 1.57f,  1.5f,  1.57f, 1.5f,  3.14f,  1.57f,  1.57f},
};
float max_vel[14] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; //Rad/s

//Gripper related
enum GripperCmd
{
    BUTTON_A_FALSE = 0,
    BUTTON_A_TRUE
};
const int16_t GripperJointCmd[2][6] = {
    {1000, 1000, 1000, 1000, 1000, 0},
    {0, 0, 0, 0, 0, 0}
};
GripperCmd current_gripper_cmd[2] = {GripperCmd::BUTTON_A_FALSE};
int16_t gripper_speed[6] = {1000, 1000, 1000, 1000, 1000, 1000};
int16_t gripper_force[6] = {300, 300, 300, 300, 300, 300};
int hand_ids[2] = {robot_config.hands[0].id, robot_config.hands[1].id};
int hand_channel[2] = {robot_config.motors[0].can_channel, robot_config.motors[1].can_channel};
InspireHand hands(hand_ids, hand_channel);

//Global params
int delay_ms = 25;
hint32 target_pulse[NUM_JOINTS];
float position_error[NUM_JOINTS];
float read_position[NUM_JOINTS];
float set_velocity[NUM_JOINTS] = {0};
float read_velocity[NUM_JOINTS];
float kp_velocity[NUM_JOINTS];
float max_velocity_bound[NUM_JOINTS];
float sign[NUM_JOINTS];
float gear_ratio[NUM_JOINTS];

void sigint_handler(int s){
    printf("Caught signal %d\n", s);
    shutdown_all = true;
    // exit(0);
}

template <typename T>
bool is_equal(const T* v1, const T* v2, int n, float precision=1e-3){
    if (sizeof(v1) != sizeof(v2))
        return false;
    if (sizeof(v1) == 0 && sizeof(v2) == 0)
        return true;

    for(int i=0; i<n; i++){
        if (fabs(v1[i] - v2[i]) > precision){
            return false;
        }
    }

    return true;
}

void receive_udp_task(){
    long t0, t1;
    t0 = get_time_now();
    int sleep_us = 0.01 * 1e6;
    teleop_command temp;
    while (!shutdown_all)
    {
        if (udp_select(&udp, 50))
        {
            udp_unpack_teleop_command(&udp, &temp);
            {
                std::lock_guard<std::mutex> lock(receive_udp_mtx);
                copy_teleop_command(&temp, &received_command);
            }
        }

        t1 = get_time_now();
        long sleep_time = (t1 - t0) - sleep_us;
        if (sleep_time > 0)
        {
            usleep(sleep_time);
        }
        t0 = get_time_now();
    }
    printf("Exit receive_udp_task\n");
}

void set_robot_command(EyouJoints* arm){
    if (shutdown_all) return;

    //Set arm command
    std::cout << "arm_cmd:";
    for (int j=0; j<NUM_JOINTS; j++){   
        std::cout << teleop_command_.JointPosCmd[j] << ", ";
    }
    std::cout <<"\n";

    for (int arm_id = 0; arm_id < NUM_ARM; ++arm_id)
    {
        for (int joint_id = 0; joint_id < NUM_JOINTS_PER_ARM; ++joint_id)
        {
            int global_joint_index = arm_id * NUM_JOINTS_PER_ARM + joint_id;
            if (!robot_config.motors[arm_id].enabled[joint_id])
                continue;
            float target_pos = teleop_command_.JointPosCmd[global_joint_index];

            //position limit
            target_pos = std::max(target_pos, pos_limits[0][global_joint_index]);
            target_pos = std::min(target_pos, pos_limits[1][global_joint_index]);
            
            //velocity limit
            float max_vel_step = max_vel[0] / 2; 
            float kp = 10.0;
            
            //velocity control
            float pos_error = target_pos - read_position[global_joint_index];
            float vel = pos_error * kp;
            float vel_bound = std::min(std::max(vel, -max_vel[global_joint_index]), max_vel[global_joint_index]);
            float acc_bound = std::min(std::max(vel_bound, read_velocity[global_joint_index]-max_vel_step), read_velocity[global_joint_index]+max_vel_step);
            float motor_vel = acc_bound * RAD2DEGREE * gear_ratio[global_joint_index] * sign[global_joint_index];
            arm[arm_id].set_velocity(joint_id, motor_vel);
            usleep(100);

            set_velocity[global_joint_index] = acc_bound;
            position_error[global_joint_index] = pos_error;
            kp_velocity[global_joint_index] = vel;
            max_velocity_bound[global_joint_index] = vel_bound;


            // velocity_factor = 0.5*(1.0 - std::cos(M_PI * (current_t) / T));
        }
    }
    
    // // Set gripper command
    // for (int i=0; i < 2; i++){
    //     int16_t cmd[6];
    //     for (int j=0; j<6; j++){   
    //         cmd[j] = int16_t(teleop_command_.HandJointCmd[i*6+j] * 840);
    //         cmd[j] = std::min(cmd[j], int16_t(1000));
    //         cmd[j] = std::max(cmd[j], int16_t(0));
    //         cmd[j] = 1000 - cmd[j];
    //     }
    //     hands.setAngles(i, cmd);
    // }

    if ((int)current_gripper_cmd[0] != (int)teleop_command_.left_aButton){
        current_gripper_cmd[0] = (GripperCmd)teleop_command_.left_aButton;
    }
    if ((int)current_gripper_cmd[1] != (int)teleop_command_.right_aButton){
        current_gripper_cmd[1] = (GripperCmd)teleop_command_.right_aButton;
    }

    // std::cout << "left_aButton: " << " , " << (int)teleop_command_.left_aButton << std::endl;
    // std::cout << "right_aButton: " << " , " << (int)teleop_command_.right_aButton << std::endl;
    for (int i=0; i < 2; i++){
        hands.setAngles(i, GripperJointCmd[(int)current_gripper_cmd[i]]);
    }
}

void read_robot_state(EyouJoints* arm){
    //Read arm state
    int ret1, ret2;
    for (int i = 0; i < teleop_robot_state_.JointSize; i++)
    {
        int arm_id = i / NUM_JOINTS_PER_ARM;
        int joint_id = i % NUM_JOINTS_PER_ARM;
        if (!robot_config.motors[arm_id].enabled[joint_id])
            continue;

        read_position[i] = arm[arm_id].get_mdhpos(joint_id, &ret1) * DEGREE2RAD;
        read_velocity[i] = arm[arm_id].get_velocity(joint_id, &ret2) * DEGREE2RAD * sign[i] / gear_ratio[i];
        if (ret1 != HARMONIC_SUCCESS|| ret1 != HARMONIC_SUCCESS){
            printf("Joint %d, read position / read velocity failed! Pos: %f, Vel: %f", i, read_position[i], read_velocity[i]);
            shutdown_all = true;
        }
        teleop_robot_state_.JointPos[i] = read_position[i];
        usleep(100);
    }
    //Read gripper state
}

void safety_check(EyouJoints* arm){
    for (int i = 0; i < NUM_JOINTS; i++){
        int arm_id = i / NUM_JOINTS_PER_ARM;
        int joint_id = i % NUM_JOINTS_PER_ARM;
        if (!robot_config.motors[arm_id].enabled[joint_id])
            continue;

        float vel_error = std::abs(read_velocity[i] - set_velocity[i]);
        if (vel_error > 1.0)
        {
            printf("Joint %d velocity error too large: %f, target: %f, actual: %f\n", 
                   i, vel_error, set_velocity[i], read_velocity[i]);
            shutdown_all = true;
        }

        if (abs(read_velocity[i]) > max_vel[i] * 1.1f){
            printf("Joint %d velocity too large: %f, max: %f\n", i, read_velocity[i], max_vel[i]);
            shutdown_all = true;
        }

        float allowance = 5 * DEGREE2RAD;
        if (read_position[i] < pos_limits[0][i] - allowance || read_position[i] > pos_limits[1][i] + allowance){
            printf("Joint %d position out of range: %f, min: %f, max: %f\n", i, read_position[i], pos_limits[0][i], pos_limits[1][i]);
            shutdown_all = true;
        }
    }
}

void write_to_file(float t1, float t2){
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", set_velocity[i]);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", read_position[i]);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", read_velocity[i]);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", position_error[i]);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", kp_velocity[i]);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", max_velocity_bound[i]);
    }


    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", teleop_command_.JointPosCmd[i]);
    }

    // for (int i = 0; i < 12; i++)
    // {
    //     fprintf(fwrite_data, "%f ", teleop_command_.HandJointCmd[i]);
    // }
    fprintf(fwrite_data, "%f ", t1);
    fprintf(fwrite_data, "%f ", t2);
    fprintf(fwrite_data, "\n");
}

void load_config(){
    for (int arm_id = 0; arm_id < NUM_ARM; ++arm_id)
    {
        for (int joint_id = 0; joint_id < NUM_JOINTS_PER_ARM; ++joint_id)
        {
            int global_joint_index = arm_id * NUM_JOINTS_PER_ARM + joint_id;
            sign[global_joint_index] = robot_config.motors[arm_id].sign[joint_id];
            gear_ratio[global_joint_index] = robot_config.motors[arm_id].gear_ratios[joint_id];
        }
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sigint_handler);
    
    //Init arm
    canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    CallbackDispatcher::init();

    EyouJoints arm[NUM_ARM];
    for (int i = 0; i < NUM_ARM; ++i)
    {
        new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
                                    robot_config.motors[i].ids.data(),
                                    robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
                                    robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    }
    arm[0].init_joints(harmonic_OperateMode_CyclicSyncVelocity, robot_config.motors[0].pdo_auto_report_ms);
    load_config();

    // Init udp
    // udp_init(&udp, (char *)"127.0.0.1", (char *)"127.0.0.1", 10085, 10086);
    // udp_init(&udp, (char *)"10.30.3.91", (char *)"10.30.7.95", 10085, 10086);
    udp_init(&udp, (char *)"10.30.3.91", (char *)"10.30.13.21", 10086, 10085);
    teleop_robot_state_.JointSize = 14;
    teleop_robot_state_.HandJointSize = 12;
    std::thread receive_udp_thread(receive_udp_task);

    // Init teleop_command_
    for (int arm_id = 0; arm_id < NUM_ARM; ++arm_id)
    {
        for (int joint_id = 0; joint_id < NUM_JOINTS_PER_ARM; ++joint_id)
        {
            int global_joint_index = arm_id * NUM_JOINTS_PER_ARM + joint_id;
            if (!robot_config.motors[arm_id].enabled[joint_id])
                continue;
                
            read_position[global_joint_index] = arm[arm_id].get_mdhpos(joint_id) * DEGREE2RAD;
            read_velocity[global_joint_index] = arm[arm_id].get_velocity(joint_id) * DEGREE2RAD * sign[global_joint_index] / gear_ratio[global_joint_index];
            usleep(100);
        }
    }
    printf("Initial JointPosCmd: ");
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        teleop_command_.JointPosCmd[i] = read_position[i];
        printf("%f,", teleop_command_.JointPosCmd[i]);
    }
    teleop_command_.left_aButton = (int)current_gripper_cmd[0];
    teleop_command_.right_aButton = (int)current_gripper_cmd[1];
    printf("\n");

    // Init gripper
    for (int i = 0; i < 2; i++)
    {
        hands.setSpeed(i, gripper_speed);
        hands.setForce(i, gripper_force);
        hands.setAngles(i, GripperJointCmd[current_gripper_cmd[i]]);
    }
    usleep(1000);

    int running_cnt = 0;
    float t1 = 0, t2 = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, communication_time, next_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();

    printf("Task started.\n");
    while (!shutdown_all)
    {
        next_time = start_time + std::chrono::milliseconds(delay_ms);

        // update curobo command
        {
            std::lock_guard<std::mutex> lock(receive_udp_mtx);
            if (received_command.JointSize == teleop_robot_state_.JointSize){
                copy_teleop_command(&received_command, &teleop_command_);
            }
        }

        read_robot_state(arm);
        set_robot_command(arm);
        safety_check(arm);

        // Send robot state
        teleop_robot_state_.Timestamp = get_time_now();
        udp_pack_teleop_robot_state(&udp, &teleop_robot_state_);
        udp_send(&udp);
        communication_time = std::chrono::high_resolution_clock::now();

        write_to_file(t1 ,t2);

        std::this_thread::sleep_until(next_time);
        end_time = std::chrono::high_resolution_clock::now();
        
        t1 = std::chrono::duration<float, std::micro>(communication_time - start_time).count();
        t2 = std::chrono::duration<float, std::micro>(end_time - start_time).count();
        start_time = end_time;
        running_cnt += 1;
    }

    printf("Wait for receive_udp_thread.join()\n");
    receive_udp_thread.join();

    printf("closing fwrite_data\n");
    fclose(fwrite_data);
    
    printf("all_motor_stop() and canable_close()\n");
    for (int i = 0; i < NUM_ARM; ++i) {
        arm[i].all_motor_stop();
    }
    return 0;
}
