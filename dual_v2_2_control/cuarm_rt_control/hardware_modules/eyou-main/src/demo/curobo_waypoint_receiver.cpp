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
std::string config_path = base_path+"/config.yaml";
ConfigLoader robot_config(config_path);
EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
FILE *fwrite_data = fopen("curobo_receiver.txt", "w");

udp_node udp;
std::mutex receive_udp_mtx;
CuroboCommand received_command;

CuroboCommand curobo_command_;
DualArmState dual_arm_state_;
bool shutdown_all = false;

////////////////////////////////////////////////////////////////Gripper
bool enable_gripper[2] = {true, true};
std::string gripper_name[2] = {"InspireHand", "TwoFingerGripper"};
int current_gripper_joint_cmd[7];
const std::map<std::string, int> kGripperJointSize{
    {"InspireHand",         6},
    {"TwoFingerGripper",    2}
};

int16_t gripper_speed[6] = {1000, 1000, 1000, 1000, 1000, 1000};
int16_t gripper_force[6] = {300, 300, 300, 300, 300, 300};
InspireHand hands(robot_config.hardware.hand.ids);

int two_finger_gripper_id[2] = {-1 , 1};
std::string two_finger_gripper_com_id[2] = {"" ,"/dev/ttyUSB0"};
serial_uart_servo two_finger_gripper[2];

////////////////////////////////////////////////////////////// trajectory params
int delay_ms = 25;
double dt = delay_ms / 1000.0;
double T_segment;
std::vector<std::vector<std::vector<double>>> traj_coef;
std::vector<std::vector<double>> waypoints;
hint32 target_pulse[NUM_JOINTS];
hint32 read_pulse[NUM_JOINTS];
hint32 set_pulse[NUM_JOINTS];

float set_position[NUM_JOINTS];
float read_position[NUM_JOINTS];
float plan_velocity[NUM_JOINTS];
float plan_acceleration[NUM_JOINTS];

float prev_last_waypoint[NUM_JOINTS];
bool all_waypoints_reached = true;
int waypoints_index = -1;
int waypoints_size = 0;
double waypoint_traj_next_t = T_segment;

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
    CuroboCommand temp;
    temp.JointSize = 0;
    temp.WaypointsSize = 0;
    while (!shutdown_all)
    {
        if (udp_select(&udp, 50))
        {
            udp_unpack_curobo_command(&udp, &temp);
            {
                std::lock_guard<std::mutex> lock(receive_udp_mtx);
                copy_curobo_command(&temp, &received_command);
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

void plan_traj(){
    //Calculate coefficients for all waypoints
    waypoints = std::vector<std::vector<double>>(waypoints_size + 1, std::vector<double>(NUM_JOINTS));
    for (int j = 0; j < NUM_JOINTS; j++)
    {
        waypoints[0][j] = read_position[j];
        for (int i=0; i < waypoints_size; i++){
            waypoints[i+1][j] = curobo_command_.Waypoints[i][j];
        }
    }
    traj_coef = polynomial_trajectory::calculateTrajectoryCoefficients(waypoints, T_segment);
    // for (int ii=0; ii<waypoints.size(); ii++){
    //     std::cout << "waypoints: ";
    //     for (int i=0; i<curobo_command_.JointSize;i ++){
    //         std::cout << waypoints[ii][i] <<", ";
    //     }
    //     std::cout <<"\n";
    // }
}

void set_robot_command(){
    //Set arm command
    waypoint_traj_next_t += dt;
    waypoint_traj_next_t = (waypoint_traj_next_t > T_segment) ? T_segment : waypoint_traj_next_t;
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        if (!robot_config.hardware.motor.enabled[i])
            continue;
        double pos, vel, acc;
        std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[waypoints_index][i], waypoint_traj_next_t);
        int pulse = sign[i] * gear_ratio[i] * pos / PULSE2RAD + m_init_mdhpos[i];

        dual_arm.set_pos(i, pulse);
        usleep(100);

        set_pulse[i] = pulse;
        set_position[i] = (set_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ratio[i];
        plan_velocity[i] =  vel;
        plan_acceleration[i] = acc;
    }
}

void set_gripper_command(float* gripper_cmd){
    int round_down_multiper = 10;
    int start_ind = 0, end_ind = 0;
    for (int i = 0; i < 2; i++)
    {   
        int gripper_joint_size = (gripper_name[i] == "InspireHand")? 6 : 1;
        start_ind = end_ind;
        end_ind = start_ind + gripper_joint_size;
        if (!enable_gripper[i]) continue;

        bool need_update = false;
        int16_t cmd[gripper_joint_size];
        int min_val = (gripper_name[i] == "InspireHand")? 0 : 450;
        int max_val = (gripper_name[i] == "InspireHand")? 1000 : 3000;
        for (int j = 0; j < gripper_joint_size; j++){
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
    if (shutdown_all || curobo_command_.GripperJointSize != dual_arm_state_.GripperJointSize) return;
    set_gripper_command(curobo_command_.GripperJointCmd);
}

void read_robot_state(){
    //Read arm state
    for (int i = 0; i < dual_arm_state_.JointSize; i++)
    {
        if (!robot_config.hardware.motor.enabled[i])
            continue;

        read_pulse[i] = dual_arm.get_position_pulse(i);

        // // // Pretend it reach the target for simulation
        // read_pulse[i] = set_pulse[i];

        read_position[i] = (read_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ratio[i];
        dual_arm_state_.JointPos[i] = read_position[i];
        usleep(100);
    }
}

void safety_check(){
    for (int i=0; i < NUM_JOINTS; i++){
        if (abs(read_position[i] - set_position[i]) > 0.2)
        {
            printf("Joint %d out of range, set pos: %f, read pos: %f\n", i, set_position[i], read_position[i]);
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
    ////////////////////////////////////////Create UDP
    udp_init(&udp, (char *)"10.30.13.20", (char *)"10.30.13.21", 10185, 10186);
    dual_arm_state_.JointSize = 14;
    dual_arm_state_.GripperJointSize = 7;
    dual_arm_state_.status = (int)RobotStatus::kAvailable;

    std::thread receive_udp_thread(receive_udp_task);


    // ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    load_config();

    // float init_pos[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // float immutation_init_pos[] = {55.070700, 56.936422, -25.906014, 47.423157, 29.365563, 34.855786, -30.930951, -73.256998, 64.722697, 71.792059, 18.696942, -30.598646, 51.095444, 30.263731};
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

        // read_position[i] = pour_tea_init_pos[i] * DEGREE2RAD;
        // read_pulse[i] = sign[i] * gear_ratio[i] * read_position[i] / PULSE2RAD + m_init_mdhpos[i];
    }

    printf("Initial JointPosCmd: ");
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        curobo_command_.JointPosCmd[i] = read_position[i];
        curobo_command_.GripperJointCmd[i] = 1.0;
        set_pulse[i] = read_pulse[i];
        set_position[i] = read_position[i];
        printf("%f,", curobo_command_.JointPosCmd[i] * 180 / M_PI);
    }
    printf("\n");

    //Gripper Initialization
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
    }
    std::fill(current_gripper_joint_cmd, current_gripper_joint_cmd + dual_arm_state_.GripperJointSize, -1);
    float init_gripper_command[] = {1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0};
    set_gripper_command(init_gripper_command);

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
            if (received_command.JointSize == dual_arm_state_.JointSize && all_waypoints_reached){
                copy_curobo_command(&received_command, &curobo_command_);
            }
        }
        
        if (all_waypoints_reached && curobo_command_.WaypointsSize > 0 && 
            !is_equal(curobo_command_.Waypoints[curobo_command_.WaypointsSize-1], prev_last_waypoint, NUM_JOINTS)){ 
            printf("Received new waypoints\n");

            all_waypoints_reached = false;
            T_segment = (double)curobo_command_.WaypointsDT;
            waypoints_size  = (int)curobo_command_.WaypointsSize;
            waypoints_index = -1;
            waypoint_traj_next_t = T_segment;
            dual_arm_state_.status = (int)RobotStatus::kBusy;
            plan_traj();

            for (int i=0; i<NUM_JOINTS; i++){
                prev_last_waypoint[i] = curobo_command_.Waypoints[waypoints_size-1][i];
            }
        }

        set_curobo_gripper_command();

        if (waypoints_size > 0){
            if (waypoints_index < waypoints_size - 1 || waypoint_traj_next_t < T_segment){
                if (waypoint_traj_next_t >= T_segment){
                    waypoints_index += 1;
                    waypoint_traj_next_t = 0.0;
                }
                set_robot_command();

            }else{
                all_waypoints_reached = true;
                waypoints_size = 0;


                double l7_world[6];
                double hand_world[6];
                double quat[4];
                
                for (int arm_i=0; arm_i<2; arm_i++){
                    double q[7];
                    for (int j=0; j<7; j++){
                        q[j] = read_position[j + arm_i*7];
                    }
                    kinematics::forwardKinematics(arm_i, q, l7_world);
                    kinematics::addTool(hand_world, l7_world);
                    robotics_math::Eul2Quat(hand_world[5], hand_world[4], hand_world[3], quat);
                    int ii = waypoints_size-1;
                    std::cout <<"arm_"<<arm_i<<" last waypoint, "; 
                    std::cout <<"q_"<<arm_i<<": " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5] << ", " << q[6] << std::endl;
                    std::cout <<"l7_world(x,y,z,qw,qx,qy,qz): " << l7_world[0] << ", " << l7_world[1] << ", " << l7_world[2] << ", " << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << std::endl;
                    std::cout <<"hand_world(x,y,z,qw,qx,qy,qz): " << hand_world[0] << ", " << hand_world[1] << ", " << hand_world[2] << ", " << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << std::endl;
                }
                dual_arm_state_.status = (int)RobotStatus::kAvailable;
                printf("------------------------------All Waypoints finished------------------------\n");
            }
        }

        read_robot_state();
        // safety_check();

        // Send robot state
        udp_pack_dual_arm_state(&udp, &dual_arm_state_);
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
            fprintf(fwrite_data, "%f ", plan_velocity[i]);
        }
        
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            fprintf(fwrite_data, "%f ", plan_acceleration[i]);
        }

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            fprintf(fwrite_data, "%f ", curobo_command_.Waypoints[waypoints_index][i]);
        }
        fprintf(fwrite_data, "%d ", waypoints_index);
        fprintf(fwrite_data, "%lf ", waypoint_traj_next_t);
        fprintf(fwrite_data, "%f ", t1);
        fprintf(fwrite_data, "%f ", t2);
        fprintf(fwrite_data, "\n");

        start_time = end_time;
    }

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
