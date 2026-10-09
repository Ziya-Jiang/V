#include "inspire_hand.h"
#include "eyou_joints.h"
#include <unistd.h>
uint32_t motor_id[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
bool enable_motor_index[14] = {1, 1, 1, 1, 1, 1, 1,  
                               1, 1, 1, 1, 1, 1, 1}; // indicate which joint is enbled
EyouJoints dual_arm(motor_id, enable_motor_index);
int gear_ratio[14] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
int sign[14] = {1, -1, 1, 1, 1, 1, 1,
                1, 1, 1, -1, 1, 1, 1};

FILE *fwrite_data = fopen("data.txt", "w");
#define NUM_JOINTS 14
hint32 act_pos_pulse[14];
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
    // (-3909510 + static_cast<int> ((-10.557 / PULSE2DEG) * gear_ratio[1])),
    // -3909510 + static_cast<int>((-10.557 / PULSE2DEG) * gear_ratio[1]),
    -1458598,
    1063830 + static_cast<int>((-2.855 / PULSE2DEG) * gear_ratio[2]),
    1771340 + static_cast<int>((-2.462 / PULSE2DEG) * gear_ratio[3]),
    -1520960 + static_cast<int>((-3.738 / PULSE2DEG) * gear_ratio[4]),
    1999800 + static_cast<int>((-4.831 / PULSE2DEG) * gear_ratio[5]),
    1965460 + static_cast<int>((5.855 / PULSE2DEG) * gear_ratio[6])};
// 主函数

void signal_handler(int signal){
    exit_requested = true;
}

int main()
{
    signal(SIGINT, signal_handler);
    dual_arm.canable_init();
    dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    //-----------------------------------------------------
    enum GripperCmd{OPEN = 0, CLOSE};
    const int16_t GripperJointCmd[2][6] = {
        {1000, 1000, 1000, 1000, 1000, 0}, 
        {800, 800, 820, 800, 800, 0}};
    GripperCmd cur_gripper_cmd = GripperCmd::CLOSE;
    InspireHand hands(0x01, 0x02);

    int num_joints = 14;
    hint32 start_pos_pulse[num_joints];
    for (int i = 0; i < num_joints; ++i)
    {
        if (!enable_motor_index[i])
            continue;
        start_pos_pulse[i] = dual_arm.get_position_pulse(i); // start actual pulse of joints
        if((start_pos_pulse[i]-m_init_mdhpos[i])*PULSE2RAD*sign[i]/gear_ratio[i]>=M_PI)
        {
            m_init_mdhpos[i]=m_init_mdhpos[i]+2*M_PI*gear_ratio[i]*sign[i]/PULSE2RAD;
        }
        else if((start_pos_pulse[i]-m_init_mdhpos[i])*PULSE2RAD*sign[i]/gear_ratio[i]<=-M_PI)
        {
            m_init_mdhpos[i]=m_init_mdhpos[i]-2*M_PI*gear_ratio[i]*sign[i]/PULSE2RAD;
        }
        usleep(100);
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto next_time = start;
    int cnt = 0;
    
    while (!exit_requested){
        next_time = end + std::chrono::milliseconds(ITPV);
        // std::cout << cnt <<"\n";

        //Set gripper cmd
        auto t0 = std::chrono::high_resolution_clock::now();
        if (cnt % 500 == 0){
            // std::cout <<"set gripper cmd\n";
            cur_gripper_cmd = (cur_gripper_cmd == GripperCmd::OPEN)? GripperCmd::CLOSE : GripperCmd::OPEN;
            for (int i = 0; i < 2; i++){
                hands.setAngles(i, GripperJointCmd[(int)cur_gripper_cmd]);
                usleep(100);
            }
        }
        auto t1 = std::chrono::high_resolution_clock::now();

        //Set joint cmd
        // std::cout <<"set joint cmd\n";
        for (int i = 0; i < num_joints; ++i){
            if (!enable_motor_index[i])
                continue;
            dual_arm.set_pos(i, start_pos_pulse[i]);
            usleep(100);
        }
        auto t2 = std::chrono::high_resolution_clock::now();

        //Read joint cmd
        // std::cout <<"Read joint cmd\n";
        int abnormal_read = 0;
        float motor_read_delay[num_joints];
        for (int i = 0; i < num_joints; ++i){
            if (!enable_motor_index[i])
                continue;
            
            auto t21 = std::chrono::high_resolution_clock::now();
            int pulse = dual_arm.get_position_pulse(i);
            auto t22 = std::chrono::high_resolution_clock::now();
            motor_read_delay[i] = std::chrono::duration<float, std::micro>(t22 - t21).count();
            if (pulse == 255){
                abnormal_read += 1;
            }
            usleep(100);
        }
        auto t3 = std::chrono::high_resolution_clock::now();

        std::this_thread::sleep_until(next_time);
        auto t4 = std::chrono::high_resolution_clock::now();
        cnt += 1;

        fprintf(fwrite_data, "%f ", std::chrono::duration<float, std::micro>(t1 - t0).count());
        fprintf(fwrite_data, "%f ", std::chrono::duration<float, std::micro>(t2 - t0).count());
        fprintf(fwrite_data, "%f ", std::chrono::duration<float, std::micro>(t3 - t0).count());
        fprintf(fwrite_data, "%f ", std::chrono::duration<float, std::micro>(t4 - t0).count());
        fprintf(fwrite_data, "%d ", abnormal_read);
        for (int i = 0; i < num_joints; ++i){
            fprintf(fwrite_data, "%f ", motor_read_delay[i]);
        }
        fprintf(fwrite_data, "\n");
        end = std::chrono::high_resolution_clock::now();
    }
    dual_arm.canable_close();
    fclose(fwrite_data);
    return 0;
}