#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#include <unistd.h>
#include <mutex>
#include <fstream>
#include <vector>
#include <sstream>

#include "kinect.h"
#include "usb_cam.h"
#include "torque_sensor.h"
#include "eyou_joints.h"
#include "kinematics.h"
#include "inspire_hand.h"
#include "polynomial_trajectory.h"


#define CLAMP(x,low_lim,high_lim) ((x)<(low_lim)?low_lim:((x)>(high_lim)?(high_lim):(x)))
//#define PULSE2DEG 360/65536 
uint32_t motor_id[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
bool enable_motor_index[14] = {0, 0, 0, 0, 0, 0, 0, //1, 1, 1, 1, 1, 1, 1, 
                                1, 1, 1, 1, 1, 1, 1};//indicate which joint is enbled
uint32_t sensor_id[14] = {0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28};
double left_joint_pos[7] = {0};
double right_joint_pos[7] = {0};
EyouJoints dual_arm(motor_id, enable_motor_index);
hint32 joint_pos_pulse[14];
FILE *fwrite_data = fopen("data.txt", "wb"); // save data
int sign[14] = {1, -1, 1, 1, 1, 1, 1,
                1, 1, 1, -1, 1, 1, 1};
int gear_ration[14] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
bool exit_requested = false;
bool start_recording = false;
int camera_wrote_frame[2] = {-1, -1};
Kinect *kinect_ptr;
UsbCam *usb_ptr;
std::mutex wrote_frame_mtx;
int gripper_state[2] = {0, 0};
std::mutex gripper_state_mtx;
float camera_update_time[2] = {0, 0};
float camera_wrote_time[2] = {0, 0};
float camera_cycle_time[2] = {0, 0};


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

void signal_handler(int signal)
{
    exit_requested = true;
}

void kinect_task(){
    int delay_ms = 40;
    int fps = 1/(delay_ms/1000.0);
    cv::Mat frame;
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto next_time = start;
    
    int id = 0;
    kinect_ptr->startVideoWriter("Kinect", fps);
    while (!exit_requested){
        next_time = end + std::chrono::milliseconds(delay_ms);
        
        kinect_ptr->update();
        frame = kinect_ptr->getColorImage();
        auto t2 = std::chrono::high_resolution_clock::now();
        float update_time = std::chrono::duration<float, std::micro>(t2 - start).count();

        if (start_recording){
            kinect_ptr->writeVideoFrame(frame);
            auto t4 = std::chrono::high_resolution_clock::now();
            float wrote_time = std::chrono::duration<float, std::micro>(t4 - start).count();

            std::lock_guard<std::mutex> lock(wrote_frame_mtx);
            camera_wrote_frame[id] += 1;
            camera_update_time[id] = update_time;
            camera_wrote_time[id] = wrote_time;
        }

        std::this_thread::sleep_until(next_time);
        end = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(wrote_frame_mtx);
            camera_cycle_time[id] = std::chrono::duration<float, std::micro>(end - start).count();
        }
        start = std::chrono::high_resolution_clock::now();
    }
    kinect_ptr->stopVideoWriter();
}

void usb_task(){
    int delay_ms = 40;
    int fps = 1/(delay_ms/1000.0);
    cv::Mat frame;
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto next_time = start;

    int id = 1;
    usb_ptr->startVideoWriter("UsbCam", fps);
    while (!exit_requested){
        next_time = end + std::chrono::milliseconds(delay_ms);

        frame = usb_ptr->getColorImage();
        auto t2 = std::chrono::high_resolution_clock::now();
        float update_time = std::chrono::duration<float, std::micro>(t2 - start).count();

        if (start_recording){
            usb_ptr->writeVideoFrame(frame);
            auto t4 = std::chrono::high_resolution_clock::now();
            float wrote_time = std::chrono::duration<float, std::micro>(t4 - start).count();

            std::lock_guard<std::mutex> lock(wrote_frame_mtx);
            camera_wrote_frame[id] += 1;
            camera_update_time[id] = update_time;
            camera_wrote_time[id] = wrote_time;
        }

        std::this_thread::sleep_until(next_time);
        end = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(wrote_frame_mtx);
            camera_cycle_time[id] = std::chrono::duration<float, std::micro>(end - start).count();
        }
        start = std::chrono::high_resolution_clock::now();
    }
    usb_ptr->stopVideoWriter();
}

void xml_input()
{
    // Left Arm
    // double left_a[AXISNUM]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 62.0};
    // double left_alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD};
    // double left_d[AXISNUM]{85.0, 0.0, 250.0, 0.0, 210.0, 0.0, 0.0};
    double left_offset[AXISNUM]{0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, 0 * DEGREE2RAD, -90 * DEGREE2RAD, 0 * DEGREE2RAD};

    double left_a[AXISNUM]{0.0, -0.155, 0.128, 0.219, -0.153, -0.25, 62.172};
    double left_alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.089 * DEGREE2RAD, 90.103 * DEGREE2RAD, -89.96 * DEGREE2RAD, 90.017 * DEGREE2RAD, -90.125 * DEGREE2RAD, -89.908 * DEGREE2RAD};
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
int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGSEGV, signal_handler);
    xml_input();

    dual_arm.canable_init();
    
    //Go to initial position
    bool is_left_arm = false;
    int num_joints = 14;
    std::vector<std::vector<double>> traj_coef(num_joints);
    int T = 40;
    hint32 start_pos_pulse[num_joints];
    hint32 target_pulse[num_joints];
    double t = 0.01;
    double dt = 10/1000.f;
    float init_cmd[num_joints] = {0, 0, 0, 0, 0, 0, 0, -1.278477,1.129679,1.253070,0.326321,-0.534056,0.891978,0.528281};
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);

    for (int i = 0; i < num_joints; ++i)
    {
        if (!enable_motor_index[i])
            continue;
        start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints
        if((start_pos_pulse[i]-m_init_mdhpos[i])*PULSE2RAD*sign[i]/gear_ration[i]>=M_PI)
        {
            m_init_mdhpos[i]=m_init_mdhpos[i]+2*M_PI*gear_ration[i]*sign[i]/PULSE2RAD;
        }
        else if((start_pos_pulse[i]-m_init_mdhpos[i])*PULSE2RAD*sign[i]/gear_ration[i]<=-M_PI)
        {
            m_init_mdhpos[i]=m_init_mdhpos[i]-2*M_PI*gear_ration[i]*sign[i]/PULSE2RAD;
        }        
    }

    // Set the joint position
    printf("SET to inti pos\n");
    for (size_t i = 0; i < num_joints; i++)
    {
        if (!enable_motor_index[i])
            continue;
        start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints

        hint32 mdh_target_pulse = static_cast<int>(init_cmd[i] / PULSE2RAD) * gear_ration[i]; // change degrees to pulse

        target_pulse[i] = (m_init_mdhpos[i] + sign[i] * mdh_target_pulse);

        traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos_pulse[i], 0.0, 0.0, target_pulse[i], 0.0, 0.0, T);
    }

    t = 0.01;
    while(t<T)
    {
        for (int i = 0; i < num_joints; ++i)
        {
            if (!enable_motor_index[i])
                continue;
            int pos, vel, acc;
            std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
            dual_arm.set_pos(i, pos);
        }
        t += dt;
        usleep(1000);
    }

    //Initialize cameras
    printf("Start camera\n");
    kinect_ptr = new Kinect();
    usb_ptr = new UsbCam(0);

    //Start threads for cameras
    std::thread kinect_thread(kinect_task);
    std::thread usb_thread(usb_task);

    //Init gripper
    printf("Init gripper\n");
    const int gripper_id = 0;
    enum GripperCmd{OPEN = 0, CLOSE};
    const int16_t GripperJointCmd[2][6] = {
        {1000, 1000, 1000, 1000, 1000, 0}, 
        {800, 800, 820, 800, 800, 0}};
    int16_t speed[6] = {1000, 1000, 1000, 1000, 1000, 1000};
    int16_t force[6] = {1000, 1000, 1000, 1000, 1000, 1000};
    GripperCmd cur_gripper_cmd = GripperCmd::OPEN;
    InspireHand hands(0x01, 0x02);
    hands.setSpeed(gripper_id, speed);
    hands.setForce(gripper_id, force);
    hands.setAngles(gripper_id, GripperJointCmd[(int)cur_gripper_cmd]);
    std::cout <<"set gripper initial pos\n";
    usleep(1000000);
    
    //Read data.txt
    std::cout <<"Read data.txt\n";
    std::ifstream file("/ssd/eyou_dual_arm/imutation_data/1/data.txt");
    std::cout <<"Read data.txt1\n";
    std::vector<std::vector<float>> all_joint_target;
    std::vector<std::vector<int>> all_gripper_command;
    std::string line;
    std::cout <<"inside0\n";
    while (std::getline(file, line)) {
        std::cout <<"inside\n";
        std::vector<float> row;
        std::stringstream ss(line);
        float num;
        
        for (int i=0; i<30; i++){
            ss >> num;
            row.push_back(num);
        }
        
        std::vector<float> joint_cmd;
        std::cout << "cmd: ";
        for (int i=0; i<14; i++){
            joint_cmd[i] = row[i];
            std::cout <<row[i] <<", ";
        }
        std::cout <<"\n";
        // joint_cmd.assign(row.begin(), row.begin() + 14);
        joint_cmd[joint_cmd.size()-2] *= -1;
        joint_cmd[joint_cmd.size()-1] *= -1;
        all_joint_target.push_back(joint_cmd);

        std::vector<int> gripper_command;
        gripper_command.push_back(static_cast<int>(row[row.size() - 2]));
        gripper_command.push_back(static_cast<int>(row[row.size() - 1]));
        all_gripper_command.push_back(gripper_command);
    }
    file.close();
    std::cout << "Total target: "<< all_joint_target.size()<<"\n";
    // T = 1;
    // dt = 10/1000.f;

    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto next_time = start;
    float cycle_time = 0;

    //Wait for user input
    std::string userInput;
    std::cout << "Enter any key to start: ";
    getline(std::cin, userInput);
    start_recording = true;

    int cmd_i = 0;
    while (cmd_i < all_joint_target.size()){
        next_time = end + std::chrono::milliseconds(ITPV);
        std::vector<float> joint_target = all_joint_target[cmd_i];
        std::cout << "cmd_i: "<< cmd_i <<"\n";
        std::cout << "cmd: ";
        for (int i = 0; i < num_joints; ++i)
        {
            if (!enable_motor_index[i])
                continue;
            int pulse = m_init_mdhpos[i] + sign[i] * (joint_target[i] / PULSE2RAD) * gear_ration[i]; // change to pulse
            // dual_arm.set_pos(i, pulse);
            std::cout <<joint_target[i] <<", ";
        }
        std::cout <<"\n";

        //set gripper
        GripperCmd next_gripper_cmd = static_cast<GripperCmd>(all_gripper_command[cmd_i][gripper_id]);
        if (cur_gripper_cmd != next_gripper_cmd){
            hands.setAngles(gripper_id, GripperJointCmd[(int)next_gripper_cmd]);
            gripper_state[gripper_id] = (int)next_gripper_cmd;
            cur_gripper_cmd = next_gripper_cmd;
        }

        //Get joint data
        for (int i = 0;i<num_joints;++i)
        {
            if (!enable_motor_index[i]) continue;
            joint_pos_pulse[i] = sign[i]*(dual_arm.get_position_pulse(i) - m_init_mdhpos[i])/gear_ration[i];
            double pos = PULSE2RAD*joint_pos_pulse[i];
            if (i < 7){
                left_joint_pos[i] = pos;
            }else{
                right_joint_pos[i - 7] = pos;
            }
        }

        //Get task data
        double left_l7_world[6];
        double right_l7_world[6];
        kinematics::forwardKinematics(0, left_joint_pos, left_l7_world);
        kinematics::forwardKinematics(1, left_joint_pos, right_l7_world);

        if (exit_requested){
            //Wait for threads to finish and save the last data
            kinect_thread.join();
            usb_thread.join();
        }

        //Write joint position
        for (int i = 0; i<7; ++i)
        {
            fprintf(fwrite_data, "%lf ", left_joint_pos[i]);
        }

        for (int i = 0; i<7; ++i)
        {
            fprintf(fwrite_data, "%lf ", right_joint_pos[i]);
        }

        //Write task position
        for (int i = 0; i<6; ++i){
            fprintf(fwrite_data, "%lf ", left_l7_world[i]);
        }
        for (int i = 0; i<6; ++i){
            fprintf(fwrite_data, "%lf ", right_l7_world[i]);
        }

        //Write camera frame
        {
            std::lock_guard<std::mutex> lock(wrote_frame_mtx);
            for (int i = 0; i < 2; ++i)
            {
                fprintf(fwrite_data, "%d ", camera_wrote_frame[i]);
            }
        }

        //Write gripper state
        {
            std::lock_guard<std::mutex> lock(gripper_state_mtx);
            for (int i = 0; i < 2; ++i)
            {
                fprintf(fwrite_data, "%d ", gripper_state[i]);
            }
        }

        fprintf(fwrite_data, "\n");

        std::this_thread::sleep_until(next_time);
        if (exit_requested) break;
        cmd_i += 1;
        // for (size_t i = 0; i < num_joints; i++)
        // {
        //     if (!enable_motor_index[i])
        //         continue;
        //     start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints

        //     hint32 mdh_target_pulse = static_cast<int>(all_joint_target[cmd_i][i] / PULSE2RAD) * gear_ration[i]; // change degrees to pulse

        //     target_pulse[i] = (m_init_mdhpos[i] + sign[i] * mdh_target_pulse);

        //     traj_coef[i] = polynomial_trajectory::calculateCoefficients(start_pos_pulse[i], 0.0, 0.0, target_pulse[i], 0.0, 0.0, T);
        // }

        // t = 0.01;
        // while(t<T)
        // {
        //     for (int i = 0; i < num_joints; ++i)
        //     {
        //         if (!enable_motor_index[i])
        //             continue;
        //         int pos, vel, acc;
        //         std::tie(pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[i], t);
        //         dual_arm.set_pos(i, pos);
        //     }
        //     t += dt;

        //     //set gripper
        //     GripperCmd next_gripper_cmd = (GripperCmd)all_gripper_command[cmd_i];
        //     if (cur_gripper_cmd != next_gripper_cmd){
        //         hands.setAngles(gripper_id, GripperJointCmd[(int)next_gripper_cmd]);
        //         gripper_state[gripper_id] = (int)next_gripper_cmd;
        //         cur_gripper_cmd = next_gripper_cmd
        //     }
    
        //     //Get joint data
        //     for (int i = 0;i<num_joints;++i)
        //     {
        //         if (!enable_motor_index[i]) continue;
        //         joint_pos_pulse[i] = sign[i]*(dual_arm.get_position_pulse(i) - m_init_mdhpos[i])/gear_ratio[i];
        //         double pos = PULSE2RAD*joint_pos_pulse[i];
        //         if (i < 7){
        //             left_joint_pos[i] = pos;
        //         }else{
        //             right_joint_pos[i - 7] = pos;
        //         }
        //     }

        //     //Get task data
        //     double left_l7_world[6];
        //     double right_l7_world[6];
        //     kinematics::forwardKinematics(0, left_joint_pos, left_l7_world);
        //     kinematics::forwardKinematics(1, left_joint_pos, right_l7_world);

        //     if (exit_requested){
        //         //Wait for threads to finish and save the last data
        //         kinect_thread.join();
        //         usb_thread.join();
        //         gripper_thread.join();
        //     }

        //     //Write joint position
        //     for (int i = 0; i<7; ++i)
        //     {
        //         fprintf(fwrite_data, "%lf ", left_joint_pos[i]);
        //     }

        //     for (int i = 0; i<7; ++i)
        //     {
        //         fprintf(fwrite_data, "%lf ", right_joint_pos[i]);
        //     }

        //     //Write task position
        //     for (int i = 0; i<6; ++i){
        //         fprintf(fwrite_data, "%lf ", left_l7_world[i]);
        //     }
        //     for (int i = 0; i<6; ++i){
        //         fprintf(fwrite_data, "%lf ", right_l7_world[i]);
        //     }

        //     //Write camera frame
        //     {
        //         std::lock_guard<std::mutex> lock(wrote_frame_mtx);
        //         for (int i = 0; i < 2; ++i)
        //         {
        //             fprintf(fwrite_data, "%d ", camera_wrote_frame[i]);
        //         }
        //         // for (int i = 0; i < 2; ++i)
        //         // {
        //         //     fprintf(fwrite_data, "%f ", camera_update_time[i]);
        //         // }
        //         // for (int i = 0; i < 2; ++i)
        //         // {
        //         //     fprintf(fwrite_data, "%f ", camera_wrote_time[i]);
        //         // }
        //         // for (int i = 0; i < 2; ++i)
        //         // {
        //         //     fprintf(fwrite_data, "%f ", camera_cycle_time[i]);
        //         // }
        //     }
        //     // fprintf(fwrite_data, "%f ", cycle_time);

        //     //Write gripper state
        //     {
        //         std::lock_guard<std::mutex> lock(gripper_state_mtx);
        //         for (int i = 0; i < 2; ++i)
        //         {
        //             fprintf(fwrite_data, "%d ", gripper_state[i]);
        //         }
        //     }

        //     fprintf(fwrite_data, "\n");

        //     std::this_thread::sleep_until(next_time);
        //     end = std::chrono::high_resolution_clock::now();
        //     cycle_time = std::chrono::duration<float, std::micro>(end - start).count();
        //     start = std::chrono::high_resolution_clock::now();
        //     if (exit_requested) break;
        // }

    }

    dual_arm.all_motor_stop();
    harmonic_freeDLL(DEVICE_INDEX);
    fclose(fwrite_data);
    return 0;
}