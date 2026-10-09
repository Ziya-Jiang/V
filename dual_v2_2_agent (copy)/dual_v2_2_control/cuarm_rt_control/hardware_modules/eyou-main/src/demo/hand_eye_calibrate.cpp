#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "eyou_joints.h"
#include "config_loader.h"
#include <math.h>
// #include "kinect.h"
bool exit_requested = false;
void signal_handler(int signal)
{
    exit_requested = true;
}
#define USE_CAM
int main()
{
    //---------------------------------------Load config-----------------------------------------------------------
    signal(SIGINT, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    FILE *fwrite_data = fopen("mdh_pos.txt", "a"); // save data
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    //--------------------------------------------------------------------------------------------------
    double dt = robot_config.force_control.dt;
    double joint_pos[NUM_JOINTS];
    double mdh_pos[NUM_JOINTS];
    double t = dt;
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    #ifdef USE_CAM
        // Kinect cam;
    #endif
    //----------------------------------------------main loop-----------------------------------------------------
    while (!exit_requested && t<robot_config.force_control.T)
    {
        std::string userInput;
        std::cout << "Enter any key to capture: ";
        getline(std::cin, userInput);
        if ((int)(t / robot_config.force_control.dt) % 10 == 0)
            std::cout << "\n=== "  << " (Time: " << std::fixed << std::setprecision(3) << t << "s) ===" << std::endl;
        next_time += std::chrono::milliseconds((int64_t)(dt * 1000));
        for (int i = 0;i<NUM_JOINTS;++i)
        {
            if (!robot_config.hardware.motor.enabled[i]) continue;
            dual_arm.set_torque(i, 0);
            joint_pos[i] = dual_arm.get_position(i);
            mdh_pos[i] = (robot_config.hardware.motor.sign[i] * 
                            (joint_pos[i] - robot_config.hardware.motor.init_pos_degrees[i]))
                            /robot_config.hardware.motor.gear_ratios[i];
            std::cout<< "Motor ID "<< i+1 << " | Pos " << joint_pos[i] << "degrees " <<std::endl;
        }
        if (robot_config.hardware.use_camera)
        {
            #ifdef USE_CAM
                // cv::Mat color;
                // for (int i=0; i<1; i++){ //Refresh n times
                //     cam.update();
                //     color = cam.getColorImage();
                //     cam.saveImage(color, std::to_string(i) + ".png");
                //     std::cout <<"Saved image " << i <<"\n";
                // }
            #endif
        }
        
        std::this_thread::sleep_until(next_time);
        t += dt;
    }
    dual_arm.all_motor_stop();
    dual_arm.canable_close();
    fprintf(fwrite_data, "{");
    for (int i = 0;i<NUM_JOINTS;++i)
    {
        if (!robot_config.hardware.motor.enabled[i]) continue;
        fprintf(fwrite_data, "%f, ", joint_pos[i]);
    }
    fprintf(fwrite_data, "},\n");
    fclose(fwrite_data);
    return 0;
}