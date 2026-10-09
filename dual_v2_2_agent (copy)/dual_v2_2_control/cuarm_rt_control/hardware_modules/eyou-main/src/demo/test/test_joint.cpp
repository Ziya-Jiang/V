#include <unistd.h>

#include "inspire_hand.h"
#include "eyou_joints.h"
#include "config_loader.h"
const int num_joints = 14;
bool exit_requested = false;

void signal_handler(int signal){
    exit_requested = true;
}


void load_config(ConfigLoader& robot_config, float* m_init_mdhpos, float* sign, float* gear_ratio){
    for (int i=0; i<num_joints; i++){
        m_init_mdhpos[i] = robot_config.hardware.motor.init_pos_pulse[i];
        sign[i] = robot_config.hardware.motor.sign[i];
        gear_ratio[i] = robot_config.hardware.motor.gear_ratios[i];
    }
}

int main()
{
    signal(SIGINT, signal_handler);
    std::string base_path = PROJECT_SOURCE_DIR;
    std::string config_path = base_path+"/config.yaml";
    ConfigLoader robot_config(config_path);
    FILE *fwrite_data = fopen("data.txt", "w");
    EyouJoints dual_arm(robot_config.hardware.motor.ids, robot_config.hardware.motor.enabled);
    InspireHand inspire_hands(robot_config.hardware.hand.ids);
    dual_arm.canable_init();
    // dual_arm.set_operate_mode(harmonic_OperateMode_CyclicSyncPosition);
    //-----------------------------------------------------
    enum GripperCmd{OPEN = 0, CLOSE};
    const int16_t GripperJointCmd[2][6] = {
        {1000, 1000, 1000, 1000, 1000, 0}, 
        {800, 800, 820, 800, 800, 0}};
    GripperCmd cur_gripper_cmd = GripperCmd::CLOSE;


    float m_init_mdhpos[num_joints];
    float sign[num_joints];
    float gear_ratio[num_joints];
    load_config(robot_config, m_init_mdhpos, sign, gear_ratio);

    hint32 start_pos_pulse[num_joints];
    for (int i = 0; i < num_joints; ++i)
    {
        if (!robot_config.hardware.motor.enabled[i])
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
        next_time = end + std::chrono::milliseconds(20);
        std::cout << cnt <<"\n";

        //Set gripper cmd
        auto t0 = std::chrono::high_resolution_clock::now();
        // if (cnt % 500 == 0){
        //     // std::cout <<"set gripper cmd\n";
        //     cur_gripper_cmd = (cur_gripper_cmd == GripperCmd::OPEN)? GripperCmd::CLOSE : GripperCmd::OPEN;
        //     for (int i = 0; i < 2; i++){
        //         inspire_hands.setAngles(i, GripperJointCmd[(int)cur_gripper_cmd]);
        //         usleep(100);
        //     }
        // }
        auto t1 = std::chrono::high_resolution_clock::now();

        //Set joint cmd
        for (int i = 0; i < num_joints; ++i){
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            dual_arm.set_pos(i, start_pos_pulse[i]);
            // usleep(10);
        }
        auto t2 = std::chrono::high_resolution_clock::now();

        //Read joint position
        int abnormal_read = 0;
        float motor_read_delay[num_joints];
        for (int i = 0; i < num_joints; ++i){
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            
            auto t21 = std::chrono::high_resolution_clock::now();
            int pulse = dual_arm.get_position_pulse(i);
            auto t22 = std::chrono::high_resolution_clock::now();
            motor_read_delay[i] = std::chrono::duration<float, std::micro>(t22 - t21).count();
            if (pulse == 255){
                abnormal_read += 1;
            }
            // usleep(10);
        }

        for (int i = 0; i < num_joints; ++i){
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            
            dual_arm.get_velocity(i);
            // usleep(10);
        }

        for (int i = 0; i < num_joints; ++i){
            if (!robot_config.hardware.motor.enabled[i])
                continue;
            
            dual_arm.get_current(i);
            // usleep(10);
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
