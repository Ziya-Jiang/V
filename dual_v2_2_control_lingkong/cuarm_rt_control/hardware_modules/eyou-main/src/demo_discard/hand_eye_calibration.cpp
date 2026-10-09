#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#include <unistd.h>

#include "kinect.h"
// #include "usb_cam.h"
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "kinematics.h"
#include "inspire_hand.h"
#include "polynomial_trajectory.h"
#define CLAMP(x, low_lim, high_lim) ((x) < (low_lim) ? low_lim : ((x) > (high_lim) ? (high_lim) : (x)))
// #define PULSE2DEG 360/65536
uint32_t motor_id[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
bool enable_motor_index[14] = {1, 1, 1, 1, 1, 1, 1,
                               0, 0, 0, 0, 0, 0, 0}; // indicate which joint is enbled
uint32_t sensor_id[14] = {0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28};
double joint_pos[14] = {0};
EyouJoints dual_arm(motor_id, enable_motor_index);
hint32 joint_pos_pulse[14];
FILE *fwrite_data = fopen("data.txt", "wb"); // save data
int sign[14] = {1, -1, 1, 1, 1, 1, 1,
                1, 1, 1, -1, 1, -1, -1};
int gear_ratio[14] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
bool exit_requested = false;
// hint32 m_init_mdhpos[14] = {
//     2326350 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
//     2963120 + static_cast<int>((0.329 / PULSE2DEG) * gear_ratio[1]),
//     1123510 + static_cast<int>((-3.788 / PULSE2DEG) * gear_ratio[2]),
//     -1727964 + static_cast<int>((-4.132 / PULSE2DEG) * gear_ratio[3]),
//     138168 + static_cast<int>((2.977 / PULSE2DEG) * gear_ratio[4]),
//     -1304367 + static_cast<int>((0.863 / PULSE2DEG) * gear_ratio[5]),
//     -48581 + static_cast<int>((5.622 / PULSE2DEG) * gear_ratio[6]),

//     1974960 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
//     // (-3909510 + static_cast<int> ((-10.557 / PULSE2DEG) * gear_ration[1])),
//     -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ratio[1]),
//     1063830 + static_cast<int>((-2.855 / PULSE2DEG) * gear_ratio[2]),
//     1771340 + static_cast<int>((-2.462 / PULSE2DEG) * gear_ratio[3]),
//     -1520960 + static_cast<int>((-3.738 / PULSE2DEG) * gear_ratio[4]),
//     1999800 + static_cast<int>((-4.831 / PULSE2DEG) * gear_ratio[5]),
//     1965460 + static_cast<int>((5.855 / PULSE2DEG) * gear_ratio[6])};
hint32 m_init_mdhpos[14] = {
    2326350 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
    2963120 + static_cast<int>((0.329 / PULSE2DEG) * gear_ratio[1]),
    1123510 + static_cast<int>((-3.788 / PULSE2DEG) * gear_ratio[2]),
    -1727964 + static_cast<int>((-4.132 / PULSE2DEG) * gear_ratio[3]),
    138168 + static_cast<int>((2.977 / PULSE2DEG) * gear_ratio[4]),
    -1304367 + static_cast<int>((0.863 / PULSE2DEG) * gear_ratio[5]),
    -48581 + static_cast<int>((5.622 / PULSE2DEG) * gear_ratio[6]),

    1974960 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
    // (-3909510 + static_cast<int> ((-10.557 / PULSE2DEG) * gear_ratio[1])),
    // -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ratio[1]),
    -1458598,
    1063830 + static_cast<int>((-2.855 / PULSE2DEG) * gear_ratio[2]),
    1771340 + static_cast<int>((-2.462 / PULSE2DEG) * gear_ratio[3]),
    -1520960 + static_cast<int>((-3.738 / PULSE2DEG) * gear_ratio[4]),
    1999800 + static_cast<int>((-4.831 / PULSE2DEG) * gear_ratio[5]),
    1965460 + static_cast<int>((5.855 / PULSE2DEG) * gear_ratio[6])};
void signal_handler(int signal)
{
    exit_requested = true;
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGSEGV, signal_handler);

    Kinect cam;
    // UsbCam cam(0);
    dual_arm.canable_init();

    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    // dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    int num_joints = 14;
    bool is_left_arm = true;
    double command[][num_joints] = {
        {38.149996, 55.953328, -37.012912, 49.394495, 32.755713, 42.868621, -34.370702, 0, 0, 0, 0, 0, 0, 0},
        {29.311502, 63.072463, -40.391205, 49.339563, 36.463596, 43.456389, -33.085303, 0, 0, 0, 0, 0, 0, 0},
        {38.682833, 80.601137, -37.304050, 49.306604, 40.605439, 43.165251, -33.107275, 0, 0, 0, 0, 0, 0, 0},
        {50.652428, 92.120293, -35.524266, 47.136806, 41.929290, 32.849097, -31.442848, 0, 0, 0, 0, 0, 0, 0},
        {53.074912, 92.048882, -35.628636, 46.378750, 20.835556, 21.895736, -51.827965, 0, 0, 0, 0, 0, 0, 0},
        {53.107871, 92.043389, -37.551242, 46.378750, 2.136839, 9.195550, -70.477243, 0, 0, 0, 0, 0, 0, 0},
        {49.097864, 93.059624, -26.081524, 52.970542, 9.250481, 24.499494, -39.797944, 0, 0, 0, 0, 0, 0, 0},
        {34.535497, 72.575630, -26.339702, 52.657432, 14.216298, 48.674891, 10.678703, 0, 0, 0, 0, 0, 0, 0},
        {22.488997, 60.847733, -33.453344, 47.285121, 16.155384, 44.879117, 11.260978, 0, 0, 0, 0, 0, 0, 0},
        {18.215319, 68.323924, -36.408664, 47.471889, 41.374481, 62.308914, 22.038558, 0, 0, 0, 0, 0, 0, 0},
        {22.774641, 68.219554, -36.661350, 28.580911, 31.168190, 47.296108, 21.884749, 0, 0, 0, 0, 0, 0, 0},
        {49.097864, 99.887621, -35.474827, 29.970681, -16.726672, 52.629966, 14.622792, 0, 0, 0, 0, 0, 0, 0},
        {45.747036, 90.823907, -35.606663, 29.575173, -18.429552, 39.462861, -15.540150, 0, 0, 0, 0, 0, 0, 0},
        {45.642666, 80.809877, -35.628636, 29.575173, 21.588119, 23.301985, -51.487389, 0, 0, 0, 0, 0, 0, 0},
        {48.279383, 72.658027, -34.601415, 29.575173, 45.758023, 29.965188, -57.150837, 0, 0, 0, 0, 0, 0, 0},
        {45.554776, 62.731887, -33.656591, 30.844093, 48.202479, 58.606524, -43.483855, 0, 0, 0, 0, 0, 0, 0},
        {42.099578, 90.060358, -30.695778, 31.558204, 40.594452, 49.031946, -34.656346, 0, 0, 0, 0, 0, 0, 0},
        {39.858369, 98.420948, -30.668312, 38.693819, 24.801617, 54.135092, -37.924777, 0, 0, 0, 0, 0, 0, 0},
        {40.566987, 98.387989, -30.706764, 38.627901, -6.064449, 33.975195, -38.149996, 0, 0, 0, 0, 0, 0, 0},
        {35.040868, 80.057314, -37.913790, 35.886814, 20.654282, 30.629860, -37.869845, 0, 0, 0, 0, 0, 0, 0},
        {39.133272, 80.106752, -37.897311, 35.886814, 22.077010, 30.437600, -37.139255, 0, 0, 0, 0, 0, 0, 0},
        {12.683706, 63.270217, 23.549177, 67.895458, -9.942620, 54.777792, -52.844199, 0, 0, 0, 0, 0, 0, 0},
        {27.383403, 74.646551, 34.150976, 75.333196, -35.738499, 33.986181, -52.377281, 0, 0, 0, 0, 0, 0, 0},
        {27.377909, 74.646551, 34.074071, 62.665969, -37.869845, 8.223261, -85.924009, 0, 0, 0, 0, 0, 0, 0},
        {27.377909, 77.722721, 25.279522, 33.469824, -9.882195, 54.805257, -57.881427, 0, 0, 0, 0, 0, 0, 0},
        {26.218853, 89.670344, 19.099717, 21.708968, -32.497535, 47.037929, -52.316856, 0, 0, 0, 0, 0, 0, 0},
        {26.213360, 83.199401, 19.116197, 29.734475, -7.695917, 75.487005, -38.754244, 0, 0, 0, 0, 0, 0, 0},
        {20.126938, 51.306114, 15.474232, 66.236523, -12.947378, 53.201255, -27.152690, 0, 0, 0, 0, 0, 0, 0},
        {20.154404, 61.968338, 25.971660, 76.184636, -21.209091, 16.215808, -55.870930, 0, 0, 0, 0, 0, 0, 0},
        {30.596901, 86.561215, -6.207271, 21.247543, 8.558343, 33.118262, -46.488613, 0, 0, 0, 0, 0, 0, 0},
        {30.618874, 87.412655, -6.146846, 20.407089, 2.219237, 47.570766, -60.117143, 0, 0, 0, 0, 0, 0, 0}};

    // bool is_left_arm = false;
    // double command[][num_joints] = {
    //     {0, 0, 0, 0, 0, 0, 0, -46.356777, 80.029848, 53.810995, 65.280713, -64.149122, 31.305519, -25.219098},
    //     {0, 0, 0, 0, 0, 0, 0, -36.403171, 80.436342, 57.420001, 71.317696, -17.880236, 0.944824, -59.414019},
    //     {0, 0, 0, 0, 0, 0, 0, -44.950528, 69.867502, 60.666459, 71.718697, 9.052728, 1.384276, -63.275710},
    //     {0, 0, 0, 0, 0, 0, 0, -42.846648, 77.843570, 70.664010, 80.040834, -50.943566, 44.022184, -73.201850},
    //     {0, 0, 0, 0, 0, 0, 0, -30.893532, 91.614922, 70.664010, 50.229455, -56.425740, 44.110075, -71.515450},
    //     {0, 0, 0, 0, 0, 0, 0, -41.704071, 90.763483, 75.223333, 13.655996, -2.345579, 46.653408, -71.526436},
    //     {0, 0, 0, 0, 0, 0, 0, -39.759492, 86.330503, 60.391801, 85.956968, -11.271964, -41.182221, -71.751656},
    //     {0, 0, 0, 0, 0, 0, 0, -46.115078, 86.270078, 74.959661, 66.401318, 23.203108, -44.950528, -71.746163},
    //     {0, 0, 0, 0, 0, 0, 0, -52.437705, 86.791928, 86.528256, 71.125436, -32.832617, 45.900845, -71.702217},
    //     {0, 0, 0, 0, 0, 0, 0, -55.041463, 99.371264, 87.687313, 91.334771, -86.385434, 58.881182, -71.702217},
    //     {0, 0, 0, 0, 0, 0, 0, -76.519719, 100.019457, 111.000284, 77.585392, -34.266332, 61.660721, -70.493722},
    //     {0, 0, 0, 0, 0, 0, 0, -55.519368, 100.041430, 96.223684, 68.362376, -50.575524, 29.443338, -70.493722},
    //     {0, 0, 0, 0, 0, 0, 0, -64.978590, 100.063403, 82.518250, 91.340264, -68.263499, 40.512055, -70.504709},
    //     {0, 0, 0, 0, 0, 0, 0, -44.148527, 78.167667, 71.070504, 70.647531, -38.380709, 38.732271, -70.504709},
    //     {0, 0, 0, 0, 0, 0, 0, -31.223121, 82.661072, 71.345162, 33.310522, -25.296002, 62.446243, -70.504709},
    //     {0, 0, 0, 0, 0, 0, 0, -51.932335, 93.372734, 86.511777, 68.620555, -54.266928, 40.495576, -70.488229},
    //     {0, 0, 0, 0, 0, 0, 0, -61.479447, 99.376758, 69.554392, 91.340264, -88.599178, 25.306988, -70.488229},
    //     {0, 0, 0, 0, 0, 0, 0, -43.522307, 76.613103, 64.967603, 68.439280, -16.446521, 18.621812, -73.300727},
    //     {0, 0, 0, 0, 0, 0, 0, -59.150347, 72.663520, 58.754839, 60.474198, 14.051503, 28.152445, -73.157905},
    //     {0, 0, 0, 0, 0, 0, 0, -52.717857, 83.419128, 68.735911, 70.306955, -16.495959, 27.894266, -73.163398},
    //     {0, 0, 0, 0, 0, 0, 0, -55.409505, 83.424621, 68.730418, 70.257516, -16.463001, 27.894266, -73.163398}
    // };
    int n = sizeof(command) / sizeof(command[0]);

    // //---------------------------------------Set 0 torque-----------------------------------------------------------
    // for (int i = 0;i<num_joints;++i)
    // {
    //     if (!enable_motor_index[i]) continue;
    //     dual_arm.set_torque(i, 0);
    // }
    std::vector<std::vector<double>> traj_coef(num_joints);
    int T = 50;
    hint32 start_pos_pulse[num_joints];
    hint32 target_pulse[num_joints];
    double dt = ITPV / 1000.f;
    double t = dt;
    // Get joint data
    for (int i = 0; i < num_joints; ++i)
    {
        if (!enable_motor_index[i])
            continue;
        joint_pos_pulse[i] = sign[i] * (dual_arm.get_position_pulse(i) - m_init_mdhpos[i]) / gear_ratio[i];
        double pos = PULSE2RAD * joint_pos_pulse[i];
        printf("%i: %lf\n", i, pos);
    }

    // Wait for user to start
    std::string userInput;
    std::cout << "Enter any key to START: ";
    getline(std::cin, userInput);

    //----------------------------------------------main loop-----------------------------------------------------
    int cmd_i = 0;
    bool Reached_goal = true;
    double joint_pos[2][7] = {0};
    float prev_joint_pos[2][7] = {0};

    // set m_init_mdhpos

    for (int i = 0; i < num_joints; ++i)
    {
        if (!enable_motor_index[i])
            continue;
        start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints
        if ((start_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ratio[i] >= M_PI)
        {
            m_init_mdhpos[i] = m_init_mdhpos[i] + 2 * M_PI * gear_ratio[i] * sign[i] / PULSE2RAD;
        }
        else if ((start_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ratio[i] <= -M_PI)
        {
            m_init_mdhpos[i] = m_init_mdhpos[i] - 2 * M_PI * gear_ratio[i] * sign[i] / PULSE2RAD;
        }
    }
    usleep(1000);

    while (!exit_requested && cmd_i < n)
    {
        if (Reached_goal)
        {
            printf("Set joint command: %d \n", cmd_i);
            // printf("CMD: ");
            for (int i = 0; i < num_joints; i++)
            {
                start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints

                hint32 mdh_target_pulse = static_cast<int>(command[cmd_i][i] / PULSE2DEG) * gear_ratio[i]; // change degrees to pulse

                target_pulse[i] = (m_init_mdhpos[i] + sign[i] * mdh_target_pulse);

                traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos_pulse[i], 0.0, 0.0, target_pulse[i], 0.0, 0.0, T);
                // printf("%lf, ", command[cmd_i][i]);
            }
            // printf("\n");

            // Set the joint position
            t = 0.01;
            while (!exit_requested && t < T)
            {
                for (int i = 0; i < num_joints; ++i)
                {
                    if (!enable_motor_index[i])
                        continue;
                    // int pulse = (command[cmd_i][i] / PULSE2RAD) * gear_ratio[i1] * sign[i1] + m_init_mdhpos[i1];
                    int pos, vel, acc;
                    std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
                    dual_arm.set_pos(i, pos);
                }
                t += dt;
                usleep(1000);
            }

            Reached_goal = false;
        }
        else
        {
            // Get joint data
            for (int i = 0; i < num_joints; ++i)
            {
                if (!enable_motor_index[i])
                    continue;
                joint_pos_pulse[i] = sign[i] * (dual_arm.get_position_pulse(i) - m_init_mdhpos[i]) / gear_ratio[i];
                double pos = PULSE2RAD * joint_pos_pulse[i];
                prev_joint_pos[i / 7][i % 7] = joint_pos[i / 7][i % 7];
                joint_pos[i / 7][i % 7] = pos;
            }

            // Compare joint difference
            int arm_i = (is_left_arm) ? 0 : 1;
            double joint_diff[7] = {0};
            double cmd_diff[7] = {0};
            Reached_goal = true;
            for (int i = 0; i < 7; ++i)
            {
                int i0 = (is_left_arm) ? 0 : 7;
                joint_diff[i] = prev_joint_pos[arm_i][i] - joint_pos[arm_i][i];
                cmd_diff[i] = command[cmd_i][i0 + i] - joint_pos[arm_i][i] * RAD2DEGREE;
                if (fabs(joint_diff[i]) > 0.01)
                {
                    Reached_goal = false;
                    break;
                }
            }

            if (Reached_goal)
            {

                usleep(5000 * 1000);

                // Get joint data
                for (int i = 0; i < num_joints; ++i)
                {
                    if (!enable_motor_index[i])
                        continue;
                    joint_pos_pulse[i] = sign[i] * (dual_arm.get_position_pulse(i) - m_init_mdhpos[i]) / gear_ratio[i];
                    double pos = PULSE2RAD * joint_pos_pulse[i];
                    prev_joint_pos[i / 7][i % 7] = joint_pos[i / 7][i % 7];
                    joint_pos[i / 7][i % 7] = pos;
                }

                // Get task data
                double l7_world[12];
                kinematics::forwardKinematics1(arm_i, joint_pos[arm_i], l7_world);

                // Write joint position
                for (int i = 0; i < 7; ++i)
                {
                    fprintf(fwrite_data, "%lf ", joint_pos[arm_i][i]);
                    // std::cout << joint_pos[arm_i][i] * RAD2DEGREE << ", ";
                }
                // std::cout << "\n";

                // Write task position
                for (int i = 0; i < 12; ++i)
                {
                    fprintf(fwrite_data, "%lf ", l7_world[i]);
                }
                fprintf(fwrite_data, "\n");

                // Capture image
                cv::Mat color;
                for (int i = 0; i < 10; i++)
                { // Refresh n times
                    cam.update();
                    color = cam.getColorImage();
                }
                cam.saveImage(color, std::to_string(cmd_i) + ".png");

                // Print joint difference and image name
                printf("cmd_i: %d, \nJoint difference: ", cmd_i);
                for (int i = 0; i < 7; ++i)
                {
                    printf("%f, ", joint_diff[i]);
                }
                printf("\n");
                // Print cmd difference and image name
                printf("Cmd difference: ");
                for (int i = 0; i < 7; ++i)
                {
                    printf("%f, ", cmd_diff[i]);
                }
                printf("\n");
                std::cout << "Saved image " << cmd_i << "\n";
                cmd_i++;
            }
        }
        usleep(ITPV * 1000);
    }

    dual_arm.all_motor_stop();
    fclose(fwrite_data);
    harmonic_freeDLL(DEVICE_INDEX);
    return 0;
}