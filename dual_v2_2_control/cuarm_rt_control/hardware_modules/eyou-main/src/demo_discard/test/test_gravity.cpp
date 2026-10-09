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
#define PULSE2DEG 0.00549316
////////////////////////////////////////////////////////////// @brief Define dual arm class
#define NUM_JOINTS 14
uint32_t motor_id[NUM_JOINTS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
bool enable_motor_index[NUM_JOINTS] = {0, 0, 0, 0, 0, 0, 0,1, 1, 1, 1, 1, 1, 1}; // 0, 0, 0, 0, 0, 0, 0}; // indicate which joint is enbled
EyouJoints dual_arm(motor_id, enable_motor_index);
int gear_ration[NUM_JOINTS] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
FILE *fwrite_data = fopen("test_gravitytest_fourier2_fan.txt", "w");
int sign[NUM_JOINTS] = {1, -1, 1, 1, 1, 1, 1,
                        1, 1, 1, -1, 1, 1, 1};

////////////////////////////////////////////////////////////// @brief Define MDH zero points and MDH target points
#define NUM_POINTS 5
// hint32 m_init_mdhpos[14] = {
//     2326350 + static_cast<int>((0. / PULSE2DEG) * gear_ration[0]),
//     2963120 + static_cast<int>((0.329 / PULSE2DEG) * gear_ration[1]),
//     1123510 + static_cast<int>((-3.788 / PULSE2DEG) * gear_ration[2]),
//     -1727964 + static_cast<int>((-4.132 / PULSE2DEG) * gear_ration[3]),
//     138168 + static_cast<int>((2.977 / PULSE2DEG) * gear_ration[4]),
//     -1304367 + static_cast<int>((0.863 / PULSE2DEG) * gear_ration[5]),
//     -48581 + static_cast<int>((5.622 / PULSE2DEG) * gear_ration[6]),

//     1974960 + static_cast<int>((0. / PULSE2DEG) * gear_ration[0]),
//     // (-3909510 + static_cast<int> ((-10.557 / PULSE2DEG) * gear_ration[1])),
//     -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ration[1]),
//     1063830 + static_cast<int>((-2.855 / PULSE2DEG) * gear_ration[2]),
//     1771340 + static_cast<int>((-2.462 / PULSE2DEG) * gear_ration[3]),
//     -1520960 + static_cast<int>((-3.738 / PULSE2DEG) * gear_ration[4]),
//     1999800 + static_cast<int>((-4.831 / PULSE2DEG) * gear_ration[5]),
//     1965460 + static_cast<int>((5.855 / PULSE2DEG) * gear_ration[6])};
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
    // -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ration[1]),
    -1458598,
    1063830 + static_cast<int>((-2.855 / PULSE2DEG) * gear_ration[2]),
    1771340 + static_cast<int>((-2.462 / PULSE2DEG) * gear_ration[3]),
    -1520960 + static_cast<int>((-3.738 / PULSE2DEG) * gear_ration[4]),
    1999800 + static_cast<int>((-4.831 / PULSE2DEG) * gear_ration[5]),
    1965460 + static_cast<int>((5.855 / PULSE2DEG) * gear_ration[6])};

double mdh_target_degrees[NUM_POINTS][NUM_JOINTS] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 30, 30},
    {34.4203, -15.6685, 59.4705,-53.0503,-59.8716,49.0358,15.0742,0,0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 30, 30, 30, 30, 30, 30, 30},
    {0, 0, 0, 0, 0, 0, 0, -30, -30, -30, -30, -30, -30, -30},
    {0, 0, 0, 0, 0, 0, 0, 30, 30, 30, 30, 30, 30, 30},
    // {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-80 , 0 , 0 , 0},
    // {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,80 , 0 , 0 , 0},
};

////////////////////////////////////////////////////////////// @brief Define pos control trajectory
double t = ITPV / 1000.0;
double dt = ITPV / 1000.0;
double T = 300;
std::vector<std::vector<double>> traj_coef(NUM_JOINTS);
hint32 target_pulse[NUM_JOINTS];
hint32 start_pos_pulse[NUM_JOINTS];
hint32 act_pos_pulse[NUM_JOINTS];
float act_current[NUM_JOINTS];
float act_pos_angle[NUM_JOINTS];

////////////////////////////////////////////////////////////// @brief Define pos control trajectory

int fourier_trajectory(double t, double pos[7])
{
    // double T = 300.0;
    double omega;
    double t1;
    t1=300.0-t;
    omega = 2 * M_PI / T;
    // double x[7][3] = {{-0.0179, -0.0126, -0.0005}, {0.0083, 0.0050, -0.0349}, {-0.0009, -0.0218, -0.0029}, 
    // {0.0102, 0.0193, -0.0046}, {0.0008, 0.0218, -0.0041}, {-0.0126, -0.0178, 0.0062}, {0.0020, -0.0057, -0.0091}};
    double x[7][3] = {{-0.0180,0.0070,0.3343}, {0.0045,-0.0046,-0.2184}, {-0.0198,-0.0056,-0.2696}, 
    {0.0258,-0.0005,-0.0235}, {0.0241,0.0020,0.0975}, {-0.0182,0.0068,0.3250}, {-0.0032,0.0021,0.1013}};
    // double pos[7];
    for (int i = 0; i < 7; i++)
    {
        // pos[i] = m_init_mdhpos[i]+ sign[i] *(x[i][0] / omega * sin(omega * t) - x[i][1] / omega * cos(omega * t) + x[i][2]) 
        // * 180.0 / M_PI/ PULSE2DEG * gear_ration[i];
        pos[i] = m_init_mdhpos[i]+sign[i] *(x[i][0] / omega * sin(omega * t1) - x[i][1] / omega * cos(omega * t1) + x[i][2]) 
        * 180.0 / M_PI/ PULSE2DEG * gear_ration[i];
        // std::cout << "T="<< T <<","<<"pos[" << i << "]=:" << (x[i][0] / omega * sin(omega * t) - x[i][1] / omega * cos(omega * t) + x[i][2]) 
        // * 180.0 / M_PI << std::endl;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
    dual_arm.canable_init();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    ///////////////////////////////////////////////////////// Main Loop
    for (int traj_idx = 0; traj_idx < 1; ++traj_idx)
    {
        auto start = std::chrono::steady_clock::now();
        auto next_time = start;
        printf("--------------------------------TRAJECTORY: %d ------------------------\n", traj_idx);
        ////////////////////////////////////////Trajectory coefficient generation
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            if (!enable_motor_index[i])
                continue;
            start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints

            // if ((start_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ration[i] >= M_PI)
            // {
            //     m_init_mdhpos[i] = m_init_mdhpos[i] + 2 * M_PI * gear_ration[i] * sign[i] / PULSE2RAD;
            // }
            // else if ((start_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ration[i] <= -M_PI)
            // {
            //     m_init_mdhpos[i] = m_init_mdhpos[i] - 2 * M_PI * gear_ration[i] * sign[i] / PULSE2RAD;
            // }

            // double temp_target_degrees[NUM_JOINTS]={34.4203, -15.6685, 59.4705,-53.0503,-59.8716,49.0358,15.0742,0,0, 0, 0, 0, 0, 0};


            hint32 mdh_target_pulse = static_cast<int>(mdh_target_degrees[traj_idx][i] / PULSE2DEG) * gear_ration[i]; // change degrees to pulse

            target_pulse[i] = (m_init_mdhpos[i] + sign[i] * mdh_target_pulse);
            // target_pulse[i] = (m_init_mdhpos[i]);

            traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos_pulse[i], 0.0, 0.0, target_pulse[i], 0.0, 0.0, 40.0);
        }

        int step = 0;
        t = ITPV / 1000.0;
        while (t <= 80.0)
        {
            step++;
            next_time += std::chrono::milliseconds(ITPV);
            if (step % 10 == 0)
                printf("\n=== Step %d (t: %.3fms) ===\n", step, t);
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                double pos, vel, acc;
                if (!enable_motor_index[i])
                    continue;
                std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
                dual_arm.set_pos(i, static_cast<int>(pos));
                act_pos_pulse[i] = dual_arm.get_position_pulse(i);
                act_current[i] = dual_arm.get_current(i);
                act_pos_angle[i] = (act_pos_pulse[i] - m_init_mdhpos[i]) / sign[i] / gear_ration[i] * PULSE2DEG;
                if (step % 1 == 0)
                {
                    printf("Joints ID: %2d | ", motor_id[i]);
                    printf("Ref pos: %7.3f pulse | ", pos);
                    printf("Act pos: %f pulse\n", act_pos_angle[i]);
                    printf("Act current: %f \n", act_current[i]);
                }

                fprintf(fwrite_data, "%f ,%f ,", act_pos_angle[i], act_current[i]);
            }

            fprintf(fwrite_data, "\n");
            std::this_thread::sleep_until(next_time);
            t += dt;
        }
        printf("--------------------------------TRAJECTORY: %d Finish------------------------\n", traj_idx);
        std::this_thread::sleep_for(std::chrono::seconds(3));
    
    //////////////////////////////////////Follow Trajectory
    step = 0;
    t = ITPV / 1000.0;
    while (t <= T)
    {
        step++;
        next_time += std::chrono::milliseconds(ITPV);
        double pos[7];
        fourier_trajectory(t, pos);
        if (step % 10 == 0)
            printf("\n=== Step %d (t: %.3fms) ===\n", step, t);

        for (int i = 0; i < NUM_JOINTS; ++i)
        {

            if (!enable_motor_index[i])
                continue;

            dual_arm.set_pos(i, static_cast<int>(pos[i]));
            act_pos_pulse[i] = dual_arm.get_position_pulse(i);
            act_current[i] = dual_arm.get_current(i);
            act_pos_angle[i] = (act_pos_pulse[i] - m_init_mdhpos[i]) / sign[i] / gear_ration[i] * PULSE2DEG;
            if (step % 1 == 0)
            {
                printf("Joints ID: %2d | ", motor_id[i]);
                printf("Ref pos: %7.3f pulse | ", pos);
                printf("Act pos: %f pulse\n", act_pos_angle[i]);
                printf("Act current: %f \n", act_current[i]);
            }

            fprintf(fwrite_data, "%f ,%f ,", act_pos_angle[i], act_current[i]);
        }

        fprintf(fwrite_data, "\n");
        std::this_thread::sleep_until(next_time);
        t += dt;
    }
    printf("--------------------------------TRAJECTORY: %d Finish------------------------\n", traj_idx);
    std::this_thread::sleep_for(std::chrono::seconds(3));
}
fclose(fwrite_data);
harmonic_freeDLL(2);
return 0;
}

// int main(int argc, char *argv[])
// {
//     ///////////////////////////////////////////////////////// Dual arm CAN INIT and set mode
//     dual_arm.canable_init();
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
//     ///////////////////////////////////////////////////////// Main Loop
//     for (int traj_idx = 0; traj_idx < NUM_POINTS; ++traj_idx)
//     {
//         auto start = std::chrono::steady_clock::now();
//         auto next_time = start;
//         printf("--------------------------------TRAJECTORY: %d ------------------------\n", traj_idx);
//         ////////////////////////////////////////Trajectory coefficient generation
//         for (int i = 0; i < NUM_JOINTS; ++i)
//         {
//             if (!enable_motor_index[i])
//                 continue;
//             start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints

//             if ((start_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ration[i] >= M_PI)
//             {
//                 m_init_mdhpos[i] = m_init_mdhpos[i] + 2 * M_PI * gear_ration[i] * sign[i] / PULSE2RAD;
//             }
//             else if ((start_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ration[i] <= -M_PI)
//             {
//                 m_init_mdhpos[i] = m_init_mdhpos[i] - 2 * M_PI * gear_ration[i] * sign[i] / PULSE2RAD;
//             }

//             hint32 mdh_target_pulse = static_cast<int>(mdh_target_degrees[traj_idx][i] / PULSE2DEG) * gear_ration[i]; // change degrees to pulse

//             target_pulse[i] = (m_init_mdhpos[i] + sign[i] * mdh_target_pulse);

//             traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos_pulse[i], 0.0, 0.0, target_pulse[i], 0.0, 0.0, T);
//         }
//         //////////////////////////////////////Follow Trajectory
//         int step = 0;
//         t = ITPV / 1000.0;
//         while (t <= T)
//         {
//             step++;
//             next_time += std::chrono::milliseconds(ITPV);
//             if (step % 10 == 0)
//                 printf("\n=== Step %d (t: %.3fms) ===\n", step, t);
//             for (int i = 0; i < NUM_JOINTS; ++i)
//             {
//                 double pos, vel, acc;
//                 if (!enable_motor_index[i])
//                     continue;
//                 std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
//                 dual_arm.set_pos(i, static_cast<int>(pos));
//                 act_pos_pulse[i] = dual_arm.get_position_pulse(i);
//                 act_current[i] = dual_arm.get_current(i);
//                 act_pos_angle[i] = (act_pos_pulse[i] - m_init_mdhpos[i]) / sign[i] / gear_ration[i] * PULSE2DEG;
//                 if (step % 1 == 0)
//                 {
//                     printf("Joints ID: %2d | ", motor_id[i]);
//                     printf("Ref pos: %7.3f pulse | ", pos);
//                     printf("Act pos: %f pulse\n", act_pos_angle[i]);
//                     printf("Act current: %f \n", act_current[i]);
//                 }

//                 fprintf(fwrite_data, "%f ,%f ,", act_pos_angle[i], act_current[i]);
//             }

//             fprintf(fwrite_data, "\n");
//             std::this_thread::sleep_until(next_time);
//             t += dt;
//         }
//         printf("--------------------------------TRAJECTORY: %d Finish------------------------\n", traj_idx);
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
//     fclose(fwrite_data);
//     harmonic_freeDLL(2);
//     return 0;
// }