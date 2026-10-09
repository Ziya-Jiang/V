#include <iostream>
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#include <unistd.h>
#include <mutex>

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

bool isInputAvailable(int timeoutSeconds) {
    fd_set set;
    struct timeval timeout;
    
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    timeout.tv_sec = timeoutSeconds;
    timeout.tv_usec = 0;

    return select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout) > 0;
}

void gripper_task(){
    const int id = 0;
    enum GripperCmd{OPEN = 0, CLOSE};
    const int16_t GripperJointCmd[2][6] = {
        {1000, 1000, 1000, 1000, 1000, 0}, 
        {800, 800, 820, 800, 800, 0}};
    int16_t speed[6] = {1000, 1000, 1000, 1000, 1000, 1000};
    int16_t force[6] = {1000, 1000, 1000, 1000, 1000, 1000};
    GripperCmd cur_cmd = GripperCmd::OPEN;
    InspireHand hands(0x01, 0x02);
    hands.setSpeed(id, speed);
    hands.setForce(id, force);
    hands.setAngles(id, GripperJointCmd[(int)cur_cmd]);
    usleep(1000000);
    
    //Wait for start recording
    while (!start_recording){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "left gripper task started" << std::endl;
    
    while (!exit_requested){
        //Wait for user input
        std::string userInput;
        std::cout << "Enter any key to set gripper: ";
        if (isInputAvailable(1)) {
            getline(std::cin, userInput);
            //Control gripper
            cur_cmd = (cur_cmd == GripperCmd::OPEN)? GripperCmd::CLOSE : GripperCmd::OPEN;
            hands.setAngles(id, GripperJointCmd[(int)cur_cmd]);
    
            //Set gripper state
            {
                std::lock_guard<std::mutex> lock(gripper_state_mtx);
                gripper_state[id] = (int)cur_cmd;
                printf("current left gripper state: %d\n", gripper_state[id]);
            }
        }

        usleep(100000);
    }
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

    //Wait for user input
    std::string userInput;
    std::cout << "Enter any key to start: ";
    getline(std::cin, userInput);
    printf("START gravity\n");
    start_recording = true;

    //Gripper task
    std::thread gripper_thread(gripper_task);

    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncTorque);
    //Set 0 torque
    for (int i = 0;i<num_joints;++i)
    {
        if (!enable_motor_index[i]) continue;
        dual_arm.set_torque(i, 0);
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto next_time = start;
    float cycle_time = 0;

    matrix::Matrix<double, 28, 1> left_param;
    matrix::Matrix<double, 28, 1> right_param;
    double left_param_temp[28] = {-0.0329, -0.0548, 0.0, 0.0,
                                  0.0306, 0.0002, -0.0548, 0.0139,
                                  -0.0225, -0.0213, -0.0001, 0.0039,
                                  0.0419, 0.0, -0.0213, 0.0005,
                                  0.0258, 0.0116, 0.0, 0.0024,
                                  0.0, 0.0095, 0.0116, -0.0018,
                                  -0.0255, -0.0006, 0.0096, 0.0019};

    double right_param_temp[28] = {-0.0289, 0.0548, 0.0, 0.0,
                                  0.0286, -0.0002, 0.0548, 0.0139,
                                  -0.0205, 0.0213, -0.0001, 0.0039,
                                  0.0359, 0.0, 0.0213, 0.0005,
                                  0.0218, -0.0116, 0.0, 0.0024,
                                  0.0, 0.0095, -0.0116, -0.0018,
                                  0.0255,0.0,0.0,0.0019};

    for (int i = 0; i < 28; i++)
    {
        left_param(i, 0) = left_param_temp[i];
        right_param(i, 0) = right_param_temp[i];
    }
    double act_pos_pulse[num_joints];
    double act_pos_angle[num_joints];
    double act_vel_rad[num_joints];
    double left_act_pos_angle[7];
    double right_act_pos_angle[7];
    int left_index = 0;
    int right_index = 1;
    matrix::Matrix<double, 7, 28> Y_G;
    matrix::Matrix<double, 7, 1> left_tau_gravity;
    matrix::Matrix<double, 7, 1> right_tau_gravity;
    matrix::Matrix<double, 14, 1> tau_gravity;
    double torque_constant[num_joints] = {0.3, -0.3, 0.3, 0.25, 0.35, 0.3, 0.3,
        0.25, 0.45, 0.15, -0.15, 0.3, 0.25, 0.3};
    // double torque_constant[num_joints] = {0.3, -0.3, 0.3, 0.25, 0.35, 0.3, 0.3,
    //     0.25, 0.35, 0.15, -0.15, -0.25, -0.25, 0.3};
  
    while (true)
    {
        next_time = end + std::chrono::milliseconds(ITPV);

        // //Get joint data
        // for (int i = 0;i<num_joints;++i)
        // {
        //     if (!enable_motor_index[i]) continue;
        //     joint_pos_pulse[i] = sign[i]*(dual_arm.get_position_pulse(i) - m_init_mdhpos[i])/gear_ratio[i];
        //     double pos = PULSE2RAD*joint_pos_pulse[i];
        //     if (i < 7){
        //         left_joint_pos[i] = pos;
        //     }else{
        //         right_joint_pos[i - 7] = pos;
        //     }
        // }

        for (int i = 0; i < num_joints; i++)
        {
            act_pos_pulse[i] = dual_arm.get_position_pulse(i);
            // act_vel_rad[i] = dual_arm.get_velocity(i);
            if ((act_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ration[i] >= M_PI)
            {
                m_init_mdhpos[i] = m_init_mdhpos[i] + 2 * M_PI * gear_ration[i] * sign[i] / PULSE2RAD;
            }
            else if ((act_pos_pulse[i] - m_init_mdhpos[i]) * PULSE2RAD * sign[i] / gear_ration[i] <= -M_PI)
            {
                m_init_mdhpos[i] = m_init_mdhpos[i] - 2 * M_PI * gear_ration[i] * sign[i] / PULSE2RAD;
            }
            // act_current[i]=dual_arm.get_current(i);
            act_pos_angle[i] = (act_pos_pulse[i] - m_init_mdhpos[i]) / sign[i] / gear_ration[i] * PULSE2RAD;
            std::cout << "act_pos_angle[" << i << "]" << act_pos_angle[i] << std::endl;
        }
        for (int i = 0; i <7; i++)
        {
            left_act_pos_angle[i] = act_pos_angle[i];
            // if(i==0||i==2||i==4||i==6){
            //     right_act_pos_angle[i] = -act_pos_angle[i + 7];
            // }
            // else
            // {
                right_act_pos_angle[i] = act_pos_angle[i + 7];
            // }
            
        }
        kinematics::computeRegression(left_index, left_act_pos_angle, Y_G);

        left_tau_gravity = Y_G * left_param;

        kinematics::computeRegression(right_index, right_act_pos_angle, Y_G);

        right_tau_gravity = Y_G * right_param;

        for (int i = 0; i < 7; i++)
        {
            tau_gravity(i, 0) = left_tau_gravity(i, 0);
        }
        for (int i = 7; i < 14; i++)
        {
            tau_gravity(i, 0) = right_tau_gravity(i - 7, 0);
        }

        for (int i = 0; i < num_joints; ++i)
        {
            std::cout << "tau_gravity[" << i << "]" << float(torque_constant[i] *tau_gravity(i, 0)) << std::endl;
            if (!enable_motor_index[i])
                continue;

            dual_arm.set_torque(i, float(torque_constant[i] * tau_gravity(i, 0)));
            // dual_arm.set_torque(i, 0);
            joint_pos_pulse[i] = dual_arm.get_position_pulse(i);
            // joint_pos[i] = (sign[i] * (joint_pos_pulse[i] - m_init_mdhpos[i])) * PULSE2DEG / gear_ration[i];
            // printf("Motor ID: %2d | ", motor_id[i]);
            // printf("Pos: %5.4d | ", joint_pos_pulse[i]);
            // printf("Pos: %5.4f degrees | ", joint_pos[i]);
            // fprintf(fwrite_data2, "%f, ", joint_pos[i]);
        }
        
        //Get task data
        double left_l7_world[6];
        double right_l7_world[6];
        kinematics::forwardKinematics(0, left_act_pos_angle, left_l7_world);
        kinematics::forwardKinematics(1, right_act_pos_angle, right_l7_world);

        if (exit_requested){
            //Wait for threads to finish and save the last data
            kinect_thread.join();
            usb_thread.join();
            gripper_thread.join();
        }

        //Write joint position
        for (int i = 0; i<7; ++i)
        {
            fprintf(fwrite_data, "%lf ", left_act_pos_angle[i]);
        }

        for (int i = 0; i<7; ++i)
        {
            fprintf(fwrite_data, "%lf ", right_act_pos_angle[i]);
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
            // for (int i = 0; i < 2; ++i)
            // {
            //     fprintf(fwrite_data, "%f ", camera_update_time[i]);
            // }
            // for (int i = 0; i < 2; ++i)
            // {
            //     fprintf(fwrite_data, "%f ", camera_wrote_time[i]);
            // }
            // for (int i = 0; i < 2; ++i)
            // {
            //     fprintf(fwrite_data, "%f ", camera_cycle_time[i]);
            // }
        }
        // fprintf(fwrite_data, "%f ", cycle_time);

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
        end = std::chrono::high_resolution_clock::now();
        cycle_time = std::chrono::duration<float, std::micro>(end - start).count();
        start = std::chrono::high_resolution_clock::now();
        if (exit_requested) break;
    }

    dual_arm.all_motor_stop();
    harmonic_freeDLL(DEVICE_INDEX);
    fclose(fwrite_data);
    return 0;
}