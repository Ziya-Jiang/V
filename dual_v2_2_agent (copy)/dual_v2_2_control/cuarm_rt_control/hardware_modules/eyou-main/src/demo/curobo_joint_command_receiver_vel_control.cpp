#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>

#include "kinematics.h"
#include "inspire_hand.h"
#include "eyou_joints.h"
#include "polynomial_trajectory.h"
#include "config_loader.h"
#include "cuarm_udp.h"
#include "cuarm_state.h"

extern "C" 
{
    #include "serial.h"     
    #include "serial_uart_servo.h" 
}

#define PULSE2DEG 0.00549316
#define NUM_JOINTS 14

std::string base_path = PROJECT_SOURCE_DIR;
std::string config_path = base_path+"/config_vla.yaml";
ConfigLoader robot_config(config_path);
EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
FILE *fwrite_data = fopen("curobo_receiver.txt", "w");
float pos_limits[2][14] = {
    {-3.2f, -1.6f, -3.2f, -1.6f, -3.2f, -1.6f, -1.6f, -3.2f, -1.6f, -3.2f, -1.6f, -3.2f, -1.6f, -1.6f},
    { 3.2f,  1.6f,  3.2f,  1.6f,  3.2f,  1.6f,  1.6f,  3.2f,  1.6f,  3.2f,  1.6f,  3.2f,  1.6f,  1.6f},
};
// float max_vel[14] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3}; //Rad/s
// float max_vel[14] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; 
float max_vel[14] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; //Rad/s

udp_node udp;
std::mutex receive_udp_mtx;
CuroboCommand* received_command_ptr = nullptr;
CuroboCommand* curobo_command_ptr = nullptr;
DualArmState dual_arm_state_;
bool shutdown_all = false;

////////////////////////////////////////////////////////////////Gripper
struct GripperInfoStruct{
    std::string name;
    int joint_size;
    std::vector<std::vector<float>> joint_hardware_limits;
    std::vector<float> initial_command;
};
const std::map<std::string, GripperInfoStruct> kGripperInfo = {
    {"InspireHand", {"InspireHand", 6, {{0, 0, 0, 0, 0, 0}, {1000, 1000, 1000, 1000, 1000, 1000}}, {1.0, 1.0, 1.0, 1.0, 1.0, 0.0}}},
    {"TwoFingerGripper", {"TwoFingerGripper", 1, {{500}, {3000}}, {1.0}}}
};
const bool enable_gripper[2] = {true, true};
const std::string gripper_name[2] = {"TwoFingerGripper", "TwoFingerGripper"};
std::vector<float> current_gripper_joint_cmd;

int16_t gripper_speed[6] = {1000, 1000, 1000, 1000, 1000, 1000};
int16_t gripper_force[6] = {300, 300, 300, 300, 300, 300};
InspireHand hands(robot_config.hardware.hand.ids);

int two_finger_gripper_id[2] = {2, 1};
std::string two_finger_gripper_com_id[2] = {"" ,"/dev/ttyUSB0"};
serial_uart_servo two_finger_gripper[2];

////////////////////////////////////////////////////////////// trajectory params
int receive_delay_ms = 20;
int delay_ms = 25;
int receiver_communication_time = 0;
int receiver_running_time = 0;

float dt = delay_ms / 1000.0;
hint32 read_pulse[NUM_JOINTS];
hint32 set_pulse[NUM_JOINTS];

float set_velocity[NUM_JOINTS] = {0};
float position_error[NUM_JOINTS];
float read_position[NUM_JOINTS];
float read_velocity[NUM_JOINTS];
float kp_velocity[NUM_JOINTS];
float max_velocity_bound[NUM_JOINTS];
float max_acceleration_bound[NUM_JOINTS];

float m_init_mdhpos[NUM_JOINTS];
float sign[NUM_JOINTS];
float gear_ratio[NUM_JOINTS];

int status = STATUS_CONNECTING;

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
    long t0, t1, t2;
    t0 = get_time_now();
    int sleep_us = receive_delay_ms * 1000;
    // CuroboCommand temp;
    // temp.JointSize = 0;
    // temp.WaypointsSize = 0;
    while (!shutdown_all)
    {
        if (udp_select(&udp, 50))
        {
            CuroboCommand temp;
            udp_unpack_curobo_command(&udp, &temp);
            {
                std::lock_guard<std::mutex> lock(receive_udp_mtx);
                // copy_curobo_command(&temp, &received_command);
                received_command_ptr = &temp;
            }
        }

        t1 = get_time_now();
        long sleep_time = (t1 - t0) - sleep_us;
        if (sleep_time > 0)
        {
            usleep(sleep_time);
        }
        t2 = get_time_now();
        t0 = t2;
        receiver_communication_time = t1 - t0;
        receiver_running_time = t2 - t0;
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

float calculate_minimum_time(float q0, float q1, float v0, float v1, float max_vel, float max_acc) {
    // Calculate distance
    float distance = std::abs(q1 - q0);

    // Time to accelerate to max velocity
    float t_accel = (max_vel - v0) / max_acc < 0 ? 0 : (max_vel - v0) / max_acc;
    // Time to decelerate from max velocity to v1
    float t_decel = (max_vel - v1) / max_acc < 0 ? 0 : (max_vel - v1) / max_acc;

    // Distance covered during acceleration
    float d_accel = v0 * t_accel + 0.5 * max_acc * t_accel * t_accel;
    // Distance covered during deceleration
    float d_decel = v1 * t_decel + 0.5 * max_acc * t_decel * t_decel;

    // Total distance covered during acceleration and deceleration
    float d_total = d_accel + d_decel;

    if (d_total >= distance) {
        // If the distance can be covered during acceleration and deceleration
        float a = 0.5 * max_acc;
        float b = v0;
        float c = -distance;

        // Calculate the discriminant
        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            throw std::runtime_error("No valid time found, check input parameters.");
        }

        // Calculate the time
        float t_total = (-b + std::sqrt(discriminant)) / (2 * a);
        return t_total;
    } else {
        // Time to reach max velocity, plus time to cover the remaining distance at max velocity
        float d_constant = distance - d_total;
        float t_constant = d_constant / max_vel;

        return t_accel + t_constant + t_decel;
    }
}

void set_robot_command(){
    if (shutdown_all || curobo_command_ptr->JointSize != NUM_JOINTS) return;
    float kp = 40.0;
    const float final_vel = 0;
    float target_pos[NUM_JOINTS];
    float max_acc[NUM_JOINTS];

    // Calculate minimum time required for each joint
    float max_T = 0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (!robot_config.hardware.motor.enabled[i])
            continue;
        target_pos[i] = curobo_command_ptr->JointPosCmd[i];
        max_acc[i] = max_vel[i] / (5 * dt); // Assuming it takes 5 time steps to reach max velocity
        float T = calculate_minimum_time(read_position[i], target_pos[i], read_velocity[i], final_vel, max_vel[i], max_acc[i]);
        max_T = std::max(max_T, T);
    }

    // Ensure all joint reach target at the same time
    int n = static_cast<int>(max_T / dt) + 1;

    //Calculate velocity
    for (int i = 0; i < NUM_JOINTS; ++i)
    {   
        if (!robot_config.hardware.motor.enabled[i])
            continue;
        
        float pos_error = target_pos[i] - read_position[i];
        float dx = (pos_error) / n;
        float kp_vel = dx * kp;
        float vel_bound = std::clamp(kp_vel, -max_vel[i], max_vel[i]);
        float acc_bound = std::clamp(vel_bound, read_velocity[i] - max_acc[i] * dt, read_velocity[i] + max_acc[i] * dt);
        float motor_vel = acc_bound * RAD2DEGREE * gear_ratio[i] * sign[i];
        dual_arm.set_velocity(i, motor_vel);
        usleep(100);

        position_error[i] = pos_error;
        kp_velocity[i] = kp_vel;
        max_velocity_bound[i] = vel_bound;
        max_acceleration_bound[i] = acc_bound;
        set_velocity[i] = acc_bound;
    }
}

void set_gripper_command(float* gripper_cmd){
    int round_down_multiper = 10;
    int start_ind = 0, end_ind = 0;
    for (int i = 0; i < 2; i++)
    {   
        int gripper_joint_size = kGripperInfo.at(gripper_name[i]).joint_size;
        start_ind = end_ind;
        end_ind = start_ind + gripper_joint_size;
        if (!enable_gripper[i]) continue;

        bool need_update = false;
        int16_t cmd[gripper_joint_size];
        for (int j = 0; j < gripper_joint_size; j++){
            float min_val = kGripperInfo.at(gripper_name[i]).joint_hardware_limits[0][j];
            float max_val = kGripperInfo.at(gripper_name[i]).joint_hardware_limits[1][j];
            // Maps normalized gripper command [0,1] to hardware range, then rounds down
            cmd[j] = int(gripper_cmd[start_ind+j] * (max_val-min_val) + min_val);
            cmd[j] = cmd[j] / round_down_multiper * round_down_multiper;
            
            //Update flag
            int prev_cmd = current_gripper_joint_cmd[start_ind+j];
            if (prev_cmd != cmd[j]) need_update |= true;

            current_gripper_joint_cmd[start_ind+j] = cmd[j];
        }

        if (need_update){
            if (gripper_name[i] == "InspireHand"){
                hands.setAngles(i, cmd);
            }else if (gripper_name[i] == "TwoFingerGripper"){
                serial_uart_servo_set_position(&two_finger_gripper[i], two_finger_gripper_id[i], (float)cmd[0], 1000);
            }
        }
    }
}

void set_curobo_gripper_command(){
    if (shutdown_all || curobo_command_ptr->GripperJointSize != dual_arm_state_.GripperJointSize) return;
    set_gripper_command(curobo_command_ptr->GripperJointCmd);
}

void read_robot_state(){
    if (shutdown_all) return;

    //Read arm state
    for (int i = 0; i < dual_arm_state_.JointSize; i++)
    {
        if (!robot_config.hardware.motor.enabled[i])
            continue;

        int ret1, ret2;
        read_pulse[i] = dual_arm.get_position_pulse(i, &ret1);
        read_velocity[i] = dual_arm.get_velocity(i, &ret2) * DEGREE2RAD * sign[i] / gear_ratio[i];
        
        if (ret1 != HARMONIC_SUCCESS || ret2 != HARMONIC_SUCCESS){
            std::cout << "Shutdown all! Read motor error: " << i << std::endl;
            shutdown_all = true;
            status = STATUS_RUNNING_QUICKSTOP;
            return;
        }

        read_position[i] = (read_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ratio[i];
        dual_arm_state_.JointPos[i] = read_position[i];
        dual_arm_state_.JointVel[i] = read_velocity[i];
        usleep(100);
    }
}

void read_gripper_state(){
    if (shutdown_all) return;
    
    int start_ind = 0, end_ind = 0;
    for (int i = 0; i < 2; i++)
    {   
        int gripper_joint_size = kGripperInfo.at(gripper_name[i]).joint_size;
        start_ind = end_ind;
        end_ind = start_ind + gripper_joint_size;
        if (!enable_gripper[i]) continue;
        
        //Read gripper value
        if (gripper_name[i] == "InspireHand"){
            if (hands.getAngles(i) != HARMONIC_SUCCESS) continue;
        }
        
        //Update dual_arm_state_.GripperJointPos
        for (int j = 0; j < gripper_joint_size; j++){
            float min_val = kGripperInfo.at(gripper_name[i]).joint_hardware_limits[0][j];
            float max_val = kGripperInfo.at(gripper_name[i]).joint_hardware_limits[1][j];
            if (gripper_name[i] == "InspireHand"){
                dual_arm_state_.GripperJointPos[start_ind+j] = (hands.joint_angle_[i][j] - min_val) / (max_val - min_val);
            }else if (gripper_name[i] == "TwoFingerGripper"){
                dual_arm_state_.GripperJointPos[start_ind+j] = (current_gripper_joint_cmd[start_ind+j] - min_val) / (max_val - min_val);
            }
        }
    }
}

void safety_check(){
    if (shutdown_all) return;

    for (int i = 0; i < NUM_JOINTS; i++){
        if (!robot_config.hardware.motor.enabled[i])
            continue;

        //Ensure Velocity follow
        float vel_error = std::abs(read_velocity[i] - set_velocity[i]);
        if (vel_error > 1.0)
        {
            printf("Joint %d velocity failed to follow: %f, target: %f, actual: %f\n", 
                   i, vel_error, set_velocity[i], read_velocity[i]);
            shutdown_all = true;
        }

        //Ensure Velocity within limit 
        if (abs(read_velocity[i]) > max_vel[i] * 1.2f){
            printf("Joint %d velocity exceed limit: %f, max: %f\n", i, read_velocity[i], max_vel[i]);
            shutdown_all = true;
        }

        //Ensure Position within limit 
        if (read_position[i] < pos_limits[0][i]|| read_position[i] > pos_limits[1][i]){
            printf("Joint %d position exceed limit: %f, min: %f, max: %f\n", i, read_position[i], pos_limits[0][i], pos_limits[1][i]);
            shutdown_all = true;
        }
    }
    if (shutdown_all) 
        status = STATUS_RUNNING_QUICKSTOP;
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
        fprintf(fwrite_data, "%f ", max_acceleration_bound[i]);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        fprintf(fwrite_data, "%f ", curobo_command_ptr->JointPosCmd[i]);
    }

    fprintf(fwrite_data, "%d ", status);
    fprintf(fwrite_data, "%f ", t1);
    fprintf(fwrite_data, "%f ", t2);
    fprintf(fwrite_data, "%d ", receiver_communication_time);
    fprintf(fwrite_data, "%d ", receiver_running_time);
    fprintf(fwrite_data, "\n");
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sigint_handler);
    ////////////////////////////////////////Create UDP
    udp_init(&udp, (char *)"10.30.13.20", (char *)"10.30.13.21", 10085, 10086);
    dual_arm_state_.JointSize = 14;
    dual_arm_state_.GripperJointSize = 7;
    xml_input();

    std::thread receive_udp_thread(receive_udp_task);


    // ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncVelocity);
    load_config();

    // float pour_tea_init_pos[] = {47.515687,55.111008,-7.917716,47.607893,18.576467,32.258198,-37.110456,-44.126451,54.929005,-0.714710,47.605556,-13.681623,31.302170,40.239657};
    float vla_init_pos[] = {0, 0, 0, 0, 0, 0, 0, -25.78310078,  34.37746771,  22.91831181,  85.94366927, -31.51267873,  22.91831181, -16.04281826};
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

        // read_position[i] = vla_init_pos[i] * DEGREE2RAD;
        // read_pulse[i] = sign[i] * gear_ratio[i] * read_position[i] / PULSE2RAD + m_init_mdhpos[i];
    }

    //Curobo command Initialization
    CuroboCommand initial_curobo_command;
    curobo_command_ptr = &initial_curobo_command;
    // curobo_command_ptr->JointSize = NUM_JOINTS;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        curobo_command_ptr->JointPosCmd[i] = read_position[i];
        curobo_command_ptr->GripperJointCmd[i] = 1.0;
        set_pulse[i] = read_pulse[i];
    }

    //Gripper Initialization
    std::vector<float> init_gripper_command;
    for (int i = 0; i < 2; i++)
    {   
        if (!enable_gripper[i]) continue;
        if (gripper_name[i] == "InspireHand"){
            hands.setSpeed(i, gripper_speed);
            hands.setForce(i, gripper_force);
        }else if (gripper_name[i] == "TwoFingerGripper"){
            if (serial_uart_servo_open(&two_finger_gripper[i], two_finger_gripper_com_id[i].c_str(), 115200) != 0) {
                fprintf(stderr, "Failed to open serial: %s\n", two_finger_gripper_com_id[i]);
                shutdown_all = true;
            }
        }

        for (int j = 0; j < kGripperInfo.at(gripper_name[i]).joint_size; j++){
            current_gripper_joint_cmd.push_back(-1.0f);
            init_gripper_command.push_back(kGripperInfo.at(gripper_name[i]).initial_command[j]);
        }
    }
    set_gripper_command(init_gripper_command.data());

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, communication_time, next_time, end_time;
    start_time = std::chrono::high_resolution_clock::now();

    printf("Initial JointPosCmd: ");
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        printf("%f,", curobo_command_ptr->JointPosCmd[i] * 180 / M_PI);
    }
    printf("\n");
    printf("Task started.\n");
    while (!shutdown_all)
    {
        next_time = start_time + std::chrono::milliseconds(delay_ms);

        // update curobo command
        {
            std::lock_guard<std::mutex> lock(receive_udp_mtx);
            if (received_command_ptr  != nullptr && received_command_ptr->JointSize == dual_arm_state_.JointSize){
                // copy_curobo_command(&received_command, &curobo_command_);
                curobo_command_ptr = received_command_ptr;
                status = STATUS_POS_VEL;
            }
        }
        
        read_robot_state();
        read_gripper_state();
        safety_check();
        set_curobo_gripper_command();
        set_robot_command();

        // Send robot state
        udp_pack_dual_arm_state(&udp, &dual_arm_state_);
        udp_send(&udp);
        communication_time = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_until(next_time);
        end_time = std::chrono::high_resolution_clock::now();
        
        float t1 = std::chrono::duration<float, std::micro>(communication_time - start_time).count();
        float t2 = std::chrono::duration<float, std::micro>(end_time - start_time).count();

        write_to_file(t1, t2);
        start_time = end_time;
    }

    printf("Stop all motors\n");
    status = STATUS_SHUTDOWN;
    for (int trial=0; trial < 5; trial++){
        printf("Set 0 velocity to all joints...\n");
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            set_velocity[i] = 0.0;
            dual_arm.set_velocity(i, set_velocity[i]);
            usleep(100);
        }
        usleep(500);
    }

    read_robot_state();
    write_to_file(0.0f, 0.0f);

    printf("Wait for receive_udp_thread.join()\n");
    receive_udp_thread.join();

    printf("closing fwrite_data\n");
    fclose(fwrite_data);
    
    printf("stop dual arm\n");
    dual_arm.canable_close();

    printf("stop gripper\n");
    for (int i = 0; i < 2; i++){   
        if (!enable_gripper[i]) continue;
        if (gripper_name[i] == "TwoFingerGripper"){
            serial_uart_servo_close(&two_finger_gripper[i]);
        }
    }
    return 0;
}
