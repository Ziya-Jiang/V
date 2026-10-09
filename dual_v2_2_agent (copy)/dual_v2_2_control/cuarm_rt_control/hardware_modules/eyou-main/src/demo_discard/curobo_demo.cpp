#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include "eyou_joints.h"
#include "polynomial_trajectory.h"
#include "cuarm_udp.h"
#include "cuarm_state.h"

#define PULSE2DEG 0.00549316
////////////////////////////////////////////////////////////// @brief Define dual arm class
uint32_t motor_id[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
bool enable_motor_index[14] = {1, 1, 1, 1, 1, 1, 1,  // 0, 0, 0, 0, 0, 0, 0
                               1, 1, 1, 1, 1, 1, 1}; // indicate which joint is enbled
EyouJoints dual_arm(motor_id, enable_motor_index);
int gear_ration[14] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
FILE *fwrite_data = fopen("dh_pos_traj.txt", "w");
#define NUM_JOINTS 14
////////////////////////////////////////////////////////////// @brief Define MDH zero points and MDH target points
#define NUM_POINTS 2
hint32 m_init_mdhpos[14] = {
    2326350 + static_cast<int>((0. / PULSE2DEG) * gear_ration[0]),
    2963120 + static_cast<int>((0.329 / PULSE2DEG) * gear_ration[1]),
    1123510 + static_cast<int>((-3.788 / PULSE2DEG) * gear_ration[2]),
    -1727964 + static_cast<int>((-4.132 / PULSE2DEG) * gear_ration[3]),
    138168 + static_cast<int>((2.977 / PULSE2DEG) * gear_ration[4]),
    -1304367 + static_cast<int>((0.863 / PULSE2DEG) * gear_ration[5]),
    -48581 + static_cast<int>((5.622 / PULSE2DEG) * gear_ration[6]),

    1974960 + static_cast<int>((0. / PULSE2DEG) * gear_ration[0]),
    // (-3909510 + static_cast<int> ((-10.557 / PULSE2DEG) * gear_ration[1])),
    -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ration[1]),
    1063830 + static_cast<int>((-2.855 / PULSE2DEG) * gear_ration[2]),
    1771340 + static_cast<int>((-2.462 / PULSE2DEG) * gear_ration[3]),
    -1520960 + static_cast<int>((-3.738 / PULSE2DEG) * gear_ration[4]),
    1999800 + static_cast<int>((-4.831 / PULSE2DEG) * gear_ration[5]),
    1965460 + static_cast<int>((5.855 / PULSE2DEG) * gear_ration[6])};
// double mdh_target_degrees[NUM_POINTS][NUM_JOINTS] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//                                                     {10, 10, 40, 10, 10, 50., 50., 10, 10, 10, 10, 10, 50., 50.}};
int sign[NUM_JOINTS] = {1, -1, 1, 1, 1, 1, 1,
                        1, 1, 1, -1, 1, -1, -1};

////////////////////////////////////////////////////////////// @brief Define pos control trajectory
double t = 0.01;
double dt = ITPV / 1000.0;
double T = 10;
std::vector<std::vector<double>> traj_coef(NUM_JOINTS);
hint32 target_pulse[NUM_JOINTS];
hint32 start_pos_pulse[NUM_JOINTS];

hint32 act_pos_pulse[NUM_JOINTS];

udp_node udp;
CuroboCommand curobo_command_;
DualArmState dual_arm_state_;
bool shutdown_all = false;

void sigint_handler(int s){
    printf("Caught signal %d\n",s);
    shutdown_all = true;
    udp_close(&udp);
    printf("A\n");
    fclose(fwrite_data);
    printf("B\n");
    harmonic_freeDLL(DEVICE_INDEX);
    printf("C\n");
    exit(0); 
}


int main(int argc, char *argv[])
{
    signal(SIGINT, sigint_handler);
    ////////////////////////////////////////Create UDP
    udp_init(&udp, (char*)"127.0.0.1", (char*)"127.0.0.1", 10085, 10086);
    dual_arm_state_.JointSize=14;
    dual_arm_state_.GripperJointSize=12;

    // ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);

    ///////////////////////////////////////////////////////// Read start abs-pos pulse

    // for (int i = 0; i < NUM_JOINTS; i++)
    // {
    //     if (!enable_motor_index[i])
    //         continue;
    //     start_pos_pulse[i] = dual_arm.get_position_pulse(i);
    //     printf("Joint id: %d | start_pos_pulse: %d\n", motor_id[i], start_pos_pulse[i]);
    // }
    
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    while (!shutdown_all)
    {
        next_time += std::chrono::milliseconds(ITPV);

        //Receive curobo command
        if (udp_select(&udp, 50)) {
            udp_unpack_curobo_command(&udp, &curobo_command_);
        }
        

        for (int i=0; i<dual_arm_state_.JointSize; i++){
            if (!enable_motor_index[i])
                continue;

            double pos = curobo_command_.JointPosCmd[i];
            double pulse = (pos / PULSE2RAD) * gear_ration[i] * sign[i] + m_init_mdhpos[i];
            dual_arm.set_pos(i, static_cast<int>(pulse));
            // dual_arm_state_.JointPos[i] = pos;
            dual_arm_state_.JointPos[i] = (dual_arm.get_position_pulse(i) - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ration[i];
            fprintf(fwrite_data, "%f, %f ", pos, dual_arm_state_.JointPos[i]);
            // printf("%lf," , dual_arm_state_.JointPos[i]);
            // printf("%lf," , pos*180/M_PI);
        }
        // printf("\n");
        //Send robot state
        udp_pack_dual_arm_state(&udp, &dual_arm_state_);
        udp_send(&udp);

        fprintf(fwrite_data, "\n");
        std::this_thread::sleep_until(next_time);
    }

    fclose(fwrite_data);
    harmonic_freeDLL(DEVICE_INDEX);
    return 0;
}
