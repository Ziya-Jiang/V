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

#include "kinematics.h"
#include "inspire_hand.h"
#include "eyou_joints.h"
#include "polynomial_trajectory.h"
#include "config_loader.h"
#include "cuarm_udp.h"
#include "cuarm_state.h"

#define PULSE2DEG 0.00549316
#define NUM_JOINTS 14

std::string base_path = PROJECT_SOURCE_DIR;
std::string config_path = base_path+"/config.yaml";
ConfigLoader robot_config(config_path);
EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
FILE *fwrite_data = fopen("curobo_receiver.txt", "w");

udp_node udp;
std::mutex receive_udp_mtx;
TeleopCommand received_command;

TeleopCommand teleop_command_;
TeleopRobotState teleop_robot_state_;
bool shutdown_all = false;


//Gripper
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
InspireHand hands(robot_config.hardware.hand.ids);

std::vector<float> start_state;

////////////////////////////////////////////////////////////// Define pos control trajectory
int delay_ms = 25;
hint32 target_pulse[NUM_JOINTS];
hint32 read_pulse[NUM_JOINTS];
hint32 set_pulse[NUM_JOINTS];

float set_position[NUM_JOINTS];
float read_position[NUM_JOINTS];

float m_init_mdhpos[NUM_JOINTS];
float sign[NUM_JOINTS];
float gear_ratio[NUM_JOINTS];

void sigint_handler(int s)
{
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

void xml_input()
{
    // Left Arm
    // double left_a[AXISNUM]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 62.0};
    // double left_alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD};
    // double left_d[AXISNUM]{85.0, 0.0, 250.0, 0.0, 210.0, 0.0, 0.0};
    double left_offset[AXISNUM]{0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, -90 * DEGREE2RAD, 0 * DEGREE2RAD};

    double left_a[AXISNUM]{0.0, -0.155, 0.128, 0.219, -0.153, -0.25, 62.172};
    double left_alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.089 * DEGREE2RAD, 90.103 * DEGREE2RAD, -89.96 * DEGREE2RAD, 90.017 * DEGREE2RAD, -90.125 * DEGREE2RAD, 89.908 * DEGREE2RAD};
    double left_d[AXISNUM]{85.0, -0.16, 250.289, -0.07, 209.883, 0.198, 0.11};
    // double left_offset[AXISNUM]{0 * DEGREE2RAD, 0.329 * DEGREE2RAD, 3.788 * DEGREE2RAD, 4.132 * DEGREE2RAD, -2.9770 * DEGREE2RAD, -90.863 * DEGREE2RAD, -5.622 * DEGREE2RAD};

    double left_sign[AXISNUM]{1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double left_base_to_world[6]{0.0, 143.31, 332.44, -60 * DEGREE2RAD, 0.0, 0.0};
    double left_init_mdhpos[AXISNUM]{
        2326350,
        2963120,
        1123510,
        -1727964,
        138168,
        -1304367,
        -48581,
    };
    for (int i = 0; i < AXISNUM; i++)
    {
        g_robot[0].m_mdh.a[i] = left_a[i];
        g_robot[0].m_mdh.alpha[i] = left_alpha[i];
        g_robot[0].m_mdh.d[i] = left_d[i];
        g_robot[0].m_mdh.offset[i] = left_offset[i];
        g_robot[0].m_mdh.sign[i] = left_sign[i];
        g_robot[0].m_mdh.m_init_mdhpos[i] = left_init_mdhpos[i];
    }
    for (int i = 0; i < 6; i++)
    {
        g_robot[0].m_mdh.base_to_world[i] = left_base_to_world[i];
    }

    // Right Arm
    // double right_a[AXISNUM]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 62.0};
    // double right_alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD};
    // double right_d[AXISNUM]{85.0, 0.0, 250.0, 0.0, 210.0, 0.0, 0.0};
    double right_offset[AXISNUM]{0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, -90 * DEGREE2RAD, 0 * DEGREE2RAD};

    double right_a[AXISNUM]{0.0, 0.258, -0.312, -0.123, -0.34, 0.135, 62.178};
    double right_alpha[AXISNUM]{0.0 * DEGREE2RAD, -89.942 * DEGREE2RAD, 90.223 * DEGREE2RAD, -89.974 * DEGREE2RAD, 90.079 * DEGREE2RAD, -90.095 * DEGREE2RAD, 89.923 * DEGREE2RAD};
    double right_d[AXISNUM]{85.0, 0.262, 250.102, 0.164, 210.318, 0.099, 0.108};
    // double right_offset[AXISNUM]{0 * DEGREE2RAD, 10.557 * DEGREE2RAD, 2.855 * DEGREE2RAD, -2.462 * DEGREE2RAD, 3.738 * DEGREE2RAD, -94.831 * DEGREE2RAD, 5.855 * DEGREE2RAD};

    double right_sign[AXISNUM]{1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0};
    double right_base_to_world[6]{0.0, -143.31, 332.44, 60 * DEGREE2RAD, 0.0, 0.0};
    double right_init_mdhpos[AXISNUM]{1974960,
                                      -3909510,
                                      1063830,
                                      1771340,
                                      -1520960,
                                      1999800,
                                      1781640 + static_cast<int>(10 / PULSE2DEG) * 101};
    for (int i = 0; i < AXISNUM; i++)
    {
        g_robot[1].m_mdh.a[i] = right_a[i];
        g_robot[1].m_mdh.alpha[i] = right_alpha[i];
        g_robot[1].m_mdh.d[i] = right_d[i];
        g_robot[1].m_mdh.offset[i] = right_offset[i];
        g_robot[1].m_mdh.sign[i] = right_sign[i];
        g_robot[1].m_mdh.m_init_mdhpos[i] = right_init_mdhpos[i];
    }
    for (int i = 0; i < 6; i++)
    {
        g_robot[1].m_mdh.base_to_world[i] = right_base_to_world[i];
    }
}

void receive_udp_task()
{
    long t0, t1;
    t0 = get_time_now();
    int sleep_us = 0.01 * 1e6;
    TeleopCommand temp;
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

void load_config(){
    for (int i=0; i<NUM_JOINTS; i++){
        m_init_mdhpos[i] = robot_config.hardware.motor.init_pos_pulse[i];
        sign[i] = robot_config.hardware.motor.sign[i];
        gear_ratio[i] = robot_config.hardware.motor.gear_ratios[i];
    }
}

bool is_distance_close(const std::vector<float> &v1, const std::vector<float> &v2) {
    float dis = 0.0f;
    for (int i = 0; i < v1.size(); ++i) {
        dis += std::fabs(v1[i]- v2[i]);
    }

    return dis < 0.5f;
}

float limit_arm_joint_command(float pos, float last_pos){
    // float max_dx = 0.010;
    float max_dx = 0.005;
    return std::min(std::max(pos, last_pos-max_dx), last_pos+max_dx);
}

void set_robot_command(){
    //Set arm command
    std::cout << std::endl << "arm_cmd:";
    for (int j=0; j<NUM_JOINTS; j++){   
        std::cout << teleop_command_.JointPosCmd[j] << ", ";
    }
    std::cout <<"\n";

    for (int i = 0; i < NUM_JOINTS; ++i)
    {   
        if (!robot_config.hardware.motor.enabled[i])
            continue;
        float pos = teleop_command_.JointPosCmd[i];
        float limits[2][14] = {
            {-1.57f, 0.8f, -1.57f, 0.8f, -3.14f, -1.57f, -1.57f, -1.57f, 0.8f, -1.57f, 0.8f, -3.14f, -1.57f, -1.57f},
            {1.57f,  1.5f,  1.57f, 1.5f,  3.14f,  1.57f,  1.57f, 1.57f,  1.5f,  1.57f, 1.5f,  3.14f,  1.57f,  1.57f},
        };
        pos = std::max(pos, limits[0][i]);
        pos = std::min(pos, limits[1][i]);
        pos = limit_arm_joint_command(pos, set_position[i]);
        int pulse = sign[i] * gear_ratio[i] * pos / PULSE2RAD + m_init_mdhpos[i];
        std::cout << "set_position: " << i << " , " << set_position[i] << std::endl;
        dual_arm.set_pos(i, pulse);
        usleep(100);

        set_position[i] = pos;
        set_pulse[i] = pulse;
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

    std::cout << "left_aButton: " << " , " << (int)teleop_command_.left_aButton << std::endl;
    std::cout << "right_aButton: " << " , " << (int)teleop_command_.right_aButton << std::endl;
    for (int i=0; i < 2; i++){
        hands.setAngles(i, GripperJointCmd[(int)current_gripper_cmd[i]]);
    }
}

void read_robot_state(){
    //Read arm state
    for (int i = 0; i < teleop_robot_state_.JointSize; i++)
    {
        if (!robot_config.hardware.motor.enabled[i])
            continue;

        read_pulse[i] = dual_arm.get_position_pulse(i);

        read_position[i] = (read_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ratio[i];
        teleop_robot_state_.JointPos[i] = read_position[i];
        // std::cout << read_position[i] << ", ";
        usleep(100);
    }
    // std::cout << std::endl;

    //Read gripper state

}

void safety_check(){
    for (int i=0; i < NUM_JOINTS; i++){
        // printf("joint_id %d , read_pos: %f, set_pose: %f\n", i, read_position[i], set_position[i]);
        if (read_position[i] - set_position[i] > 0.1)
        {
            printf("Joint %d out of range: %f, %f\n", i, read_position[i], set_position[i]);
            shutdown_all = true;
        }
    }
    
    if (shutdown_all){
        dual_arm.all_motor_stop();
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sigint_handler);
    // ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    load_config();

    ////////////////////////////////////////Create UDP
    // udp_init(&udp, (char *)"127.0.0.1", (char *)"127.0.0.1", 10085, 10086);
    // udp_init(&udp, (char *)"10.30.3.91", (char *)"10.30.7.95", 10085, 10086);
    udp_init(&udp, (char *)"10.30.3.91", (char *)"10.30.13.21", 10086, 10085);
    teleop_robot_state_.JointSize = 14;
    teleop_robot_state_.HandJointSize = 12;

    std::thread receive_udp_thread(receive_udp_task);

    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        if (!robot_config.hardware.motor.enabled[i])
            continue;
        int ret_val;
        hint32 temp_pulse = dual_arm.get_position_pulse(i, &ret_val); // start actual pulse of joints
        if (ret_val== HARMONIC_SUCCESS){
            read_pulse[i] = temp_pulse;
        }

        read_position[i] = (read_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ratio[i];
        if (read_position[i] >= M_PI)
        {
            m_init_mdhpos[i] = m_init_mdhpos[i] + 2 * M_PI * gear_ratio[i] * sign[i] / PULSE2RAD;
        }
        else if (read_position[i] <= -M_PI)
        {
            m_init_mdhpos[i] = m_init_mdhpos[i] - 2 * M_PI * gear_ratio[i] * sign[i] / PULSE2RAD;
        }
        usleep(100);
    }

    printf("Initial JointPosCmd: ");
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        teleop_command_.JointPosCmd[i] = read_position[i];
        set_pulse[i] = read_pulse[i];
        set_position[i] = read_position[i];
        printf("%f,", teleop_command_.JointPosCmd[i] * 180 / M_PI);
    }
    printf("\n");

    //Gripper Initialization
    for (int i = 0; i < 2; i++)
    {
        hands.setSpeed(i, gripper_speed);
        hands.setForce(i, gripper_force);
        hands.setAngles(i, GripperJointCmd[current_gripper_cmd[i]]);
    }
    usleep(1000);

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, communication_time, next_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();

    xml_input();

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
        
        set_robot_command();

        read_robot_state();
        // safety_check();

        // Send robot state
        teleop_robot_state_.Timestamp = get_time_now();
        udp_pack_teleop_robot_state(&udp, &teleop_robot_state_);
        udp_send(&udp);
        communication_time = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_until(next_time);
        end_time = std::chrono::high_resolution_clock::now();
        
        float t1 = std::chrono::duration<float, std::micro>(communication_time - start_time).count();
        float t2 = std::chrono::duration<float, std::micro>(end_time - start_time).count();

        //Write to file
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            fprintf(fwrite_data, "%f ", set_position[i]);
        }

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            fprintf(fwrite_data, "%f ", read_position[i]);
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

        start_time = end_time;
    }

    printf("Wait for receive_udp_thread.join()\n");
    receive_udp_thread.join();

    printf("closing fwrite_data\n");
    fclose(fwrite_data);
    
    printf("all_motor_stop() and canable_close()\n");
    // dual_arm.all_motor_stop();
    dual_arm.canable_close();
    return 0;
}
