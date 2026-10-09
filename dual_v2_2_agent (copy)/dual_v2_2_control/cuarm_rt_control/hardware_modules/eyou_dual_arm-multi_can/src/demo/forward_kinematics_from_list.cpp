#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "config_loader.h"
#include <math.h>
#include "usb_to_can.h"
#include "callback_dispatcher.h"
////////////////xiyue
bool exit_requested = false;

void signal_handler(int signal)
{
    exit_requested = true;
}

int main()
{
    signal(SIGINT, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    RobotConfig& robot_config = config_loader.getRobotConfig();
    // Initialize CAN communication and pdo callback dispatcher
    // canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    // CallbackDispatcher::init();
    // Init eyou joints class, arm[0] is left arm, arm[1] is the right arm
    // EyouJoints arm[NUM_ARM];
    double zero_pos[NUM_ARM][NUM_JOINTS_PER_ARM];

    int calibration_arm_index = 1; // 1 for right arm, 0 for left arm

    // for (int i = 0; i < NUM_ARM; ++i)
    // {
    //     new (&arm[i]) EyouJoints(i,NUM_JOINTS_PER_ARM, robot_config.motors[i].can_channel, robot_config.motors[i].use_pdo,
    //                                 robot_config.motors[i].ids.data(),
    //                                 robot_config.motors[i].enabled.data(), robot_config.motors[i].gear_ratios.data(),
    //                                 robot_config.motors[i].sign.data(), robot_config.kinematics[i].initial_position.data());
    // }
    
    int cnt = 0;
    auto start = std::chrono::steady_clock::now();
    auto next_time = start;
    double dt = 0.01;
    int T = 1000;
    int data_size = 20000;
    double joint_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double DH_pos[NUM_ARM][NUM_JOINTS_PER_ARM];
    double DH_pos_rad[NUM_ARM][NUM_JOINTS_PER_ARM];
    double ee_pos_ori[NUM_ARM][6];
    double t = dt;
    matrix::Matrix<double, 4UL, 4UL> trans;

    float trajectory[][7] = {
        {-32.507719, 88.472926, 32.238975, 20.406110, -81.337003, -18.124219, 49.691887}, 
        {-30.225878, 84.938917, 32.730260, 35.401349, -80.561978, 27.417341, 54.355906}, 
        {-30.518832, 87.405164, 35.926572, 31.419516, -79.180801, -22.561006, 54.802375}, 
        {-29.582498, 88.854405, 40.607505, 31.019766, -96.894121, 45.290669, 63.576692}, 
        {-42.982312, 98.842559, 40.718184, 26.661406, -68.609949, -34.733251, 63.214959}, 
        {-22.518566, 91.016894, 41.114671, 15.855163, -121.299035, 44.425143, 63.235735}, 
        {-30.698472, 74.343610, 42.036869, 56.380489, -79.207506, 34.776598, 50.081956}, 
        {-48.545794, 93.251203, 42.077007, 37.291432, -73.177974, -25.501152, 56.183932}, 
        {-29.931608, 84.780024, 42.153313, 20.307886, -88.020982, 9.003344, 64.121113}, 
        {-26.436415, 92.499321, 42.153748, 36.761695, -89.142892, 18.624423, 66.478385}, 
        {-42.787373, 95.165002, 42.154347, 24.850893, -89.196736, 3.604873, 45.626785}, 
        {-52.771850, 100.374197, 42.154184, 17.464768, -84.434764, 2.491121, 47.124242}, 
        {-16.148271, 86.327007, 42.154455, 20.704699, -110.848325, 20.940470, 88.540058}, 
        {-28.344448, 94.003358, 42.155217, 18.160985, -90.810528, 4.697903, 79.056090}, 
        {-24.895379, 89.580138, 42.153748, 49.245092, -85.683669, 32.414158, 52.193450}, 
        {-45.547028, 87.688947, 42.019737, 43.879743, -67.921672, -13.610419, 61.670511}, 
        {-31.551364, 87.701205, 42.020879, 19.144478, -96.843704, -1.786909, 38.018976}, 
        {-11.989177, 82.095957, 47.418426, 22.923881, -109.902468, 2.325945, 62.842294}, 
        {-44.737854, 98.446688, 47.417610, 31.031187, -73.569620, -23.077146, 63.588385}, 
        {-22.267651, 88.388576, 46.140314, 24.697030, -124.169075, 47.486084, 54.268886}, 
        {-27.062681, 98.087862, 46.025774, 38.173709, -96.768486, 23.363334, 71.192605},
    };
    int num_points = 21;

    while (cnt < num_points)
    {

        for (int j = 0;j<NUM_JOINTS_PER_ARM;++j)
        {
            DH_pos_rad[calibration_arm_index][j] = trajectory[cnt][j] * DEGREE2RAD;
        }
        kinematics::forwardKinematics(calibration_arm_index, DH_pos_rad[calibration_arm_index], ee_pos_ori[calibration_arm_index]);
        robotics_math::Eula2Mat(trans, ee_pos_ori[calibration_arm_index]);
        // std::cout << "DH_pos_rad: ";
        // for (int j = 0;j<NUM_JOINTS_PER_ARM;++j){
        //     std::cout <<  DH_pos_rad[calibration_arm_index][j] << ", ";
        // }
        // std::cout << std::endl;

        // std::cout << "ee_pos_ori: ";
        // for (int j = 0;j<6;++j){
        //     std::cout << ee_pos_ori[calibration_arm_index][j] << ", ";
        // }
        // std::cout << std::endl;

        
        std::cout << trans(0, 0) << ", " << trans(0, 1) << ", " << trans(0, 2) << ", " << trans(0, 3) << ", "
                << trans(1, 0) << ", " << trans(1, 1) << ", " << trans(1, 2) << ", " << trans(1, 3) << ", "
                << trans(2, 0) << ", " << trans(2, 1) << ", " << trans(2, 2) << ", " << trans(2, 3) << "\n";
        cnt += 1;
    }
    // arm[0].all_motor_stop();
    // arm[1].all_motor_stop();
    // canbox_init({robot_config.motors[0].can_channel, robot_config.motors[1].can_channel});
    return 0;
}