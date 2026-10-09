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



#define CLAMP(x,low_lim,high_lim) ((x)<(low_lim)?low_lim:((x)>(high_lim)?(high_lim):(x)))
//#define PULSE2DEG 360/65536 
uint32_t motor_id[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
bool enable_motor_index[14] = {0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0};//indicate which joint is enbled
uint32_t sensor_id[14] = {0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28};
double joint_pos[14] = {0};
EyouJoints dual_arm(motor_id, enable_motor_index);
hint32 joint_pos_pulse[14];
FILE *fwrite_data = fopen("data.txt", "wb"); // save data
int sign[14] = {1, -1, 1, 1, 1, 1, 1,
                1, 1, 1, -1, 1, -1, -1};
int gear_ratio[14] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
bool exit_requested = false;
hint32 m_init_mdhpos[14] = {
    2326350 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
    2963120 + static_cast<int>((0.329 / PULSE2DEG) * gear_ratio[1]),
    1123510 + static_cast<int>((-3.788 / PULSE2DEG) * gear_ratio[2]),
    -1727964 + static_cast<int>((-4.132 / PULSE2DEG) * gear_ratio[3]),
    138168 + static_cast<int>((2.977 / PULSE2DEG) * gear_ratio[4]),
    -1304367 + static_cast<int>((0.863 / PULSE2DEG) * gear_ratio[5]),
    -48581 + static_cast<int>((5.622 / PULSE2DEG) * gear_ratio[6]),

    1974960 + static_cast<int>((0. / PULSE2DEG) * gear_ratio[0]),
    // (-3909510 + static_cast<int> ((-10.557 / PULSE2DEG) * gear_ration[1])),
    -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ratio[1]),
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

    // dual_arm.canable_init();
    // dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    int num_joints = 14;
    int time_interval_us = 1000;
    int i=0;

    Kinect cam;
    // UsbCam cam(0);
    
    // //---------------------------------------Set 0 torque-----------------------------------------------------------
    // for (int i = 0;i<num_joints;++i)
    // {
    //     if (!enable_motor_index[i]) continue;
    //     dual_arm.set_torque(i, 0);
    // }

    //----------------------------------------------main loop-----------------------------------------------------
    while (!exit_requested)
    {
        //Wait for user input
        std::string userInput;
        std::cout << "Enter any key to capture: ";
        getline(std::cin, userInput);

        // //Save joint position
        // for (int i = 0;i<num_joints;++i)
        // {
        //     if (!enable_motor_index[i]) continue;
        //     joint_pos_pulse[i] = sign[i]*(dual_arm.get_position_pulse(i) - m_init_mdhpos[i])/gear_ratio[i];
        //     joint_pos[i] = PULSE2DEG*joint_pos_pulse[i];
        //     fprintf(fwrite_data, "%lf, ", joint_pos[i]);
        // }
        // fprintf(fwrite_data, "\n");

        // Capture image
        cv::Mat color;
        for (int i=0; i<10; i++){ //Refresh n times
            cam.update();
            color = cam.getColorImage();
        }
        cam.saveImage(color, std::to_string(i) + ".png");

        usleep(10);
        std::cout <<"Saved image " << i <<"\n";
        i++;
    }

    // dual_arm.all_motor_stop();
    fclose(fwrite_data);
    harmonic_freeDLL(DEVICE_INDEX);
    return 0;
}