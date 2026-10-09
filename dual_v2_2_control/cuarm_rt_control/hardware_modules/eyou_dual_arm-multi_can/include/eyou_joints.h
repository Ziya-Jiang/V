#ifndef EYOU_JOINTS_H
#define EYOU_JOINTS_H
#include <iostream>
#include "eu_harmonic.h"
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#include <unistd.h>
#include <map>
#include <mutex>
#include "eu_canable.h"
#define PULSE2DEG 0.00549316 
#define PULSE2RAD (2*M_PI/65536)
#define ITPV 2 //插补周期，单位：ms

class EyouJoints{
    public:
        EyouJoints(int arm_index = 0, int num_joints = 0, int can_channel = 0, int use_pdo = 0, uint32_t* motor_ids = nullptr, 
                    int* enable_motors = nullptr, double* gear_ratio= nullptr, int * sign = nullptr, double *init_mdh_degrees = nullptr);
        ~EyouJoints();
        int set_mdhpos(int index, double mdhpos_degree);
        int set_pos(int index, double pos_degree);
        int set_velocity(int index, double vel_degree);
        int set_torque(int index, double tor_nm);
        double get_mdhpos(huint8 index, int* ret=nullptr);
        double get_position(huint8 index, int* ret=nullptr);
        double get_velocity(huint8 index, int* ret=nullptr);
        double get_current(huint8 index,  int* ret=nullptr);  
        void fix_initial_position();
        int init_joints(int autoReportMs = 10);
        int init_joints(harmonic_OperateMode mode, int autoReportMs = 10);
        int set_operate_mode(harmonic_OperateMode mode);
        int set_operate_mode(huint8 index, harmonic_OperateMode mode);
        void all_motor_stop(int trial_times=2);
        void motor_stop(huint8 index, int trial_times=2);
        void hard_stop_all(int trial_times = 5);
        void hard_stop(huint8 index);
        static void staticReceiveCallback(int dev_index, const harmonic_CanMsg* msg);
        double mdh_to_joints(int index, const double mdh_degrees);
        double joints_to_mdh(int index, const double joints_degrees);
        std::mutex check_mutex_;
        double init_mdh_degrees_[7];
        bool enable_motors_[7];
        double zero_mdh_pos[7];
    private:
        harmonic_OperateMode operate_mode_[7];
        int num_joints_;
        int first_flag[7] = {1, 1, 1, 1, 1, 1, 1};
        int can_channel_;
        int arm_index_;  
        bool isSync_;
        int use_pdo_;
        huint8 type_;
        int check_position_following(huint8 index, double target_pos);
        int check_velocity_following(huint8 index, double target_vel);
        int check_torque_following(huint8 index, double target_tor);
        int check_position_jump(huint8 index);
        static std::map<int, EyouJoints*> instance_map_;  // arm_index到实例指针的映射
        static std::mutex map_mutex_;
        int set_pdo(int autoReportMs = 10);
        uint32_t motor_id_[7];
        double act_pos_degrees[7];
        double act_vel_dps[7];
        double act_tor_nm[7];
        
        double rated_torque_[7] = {78.0, 78.0, 8.6, 8.6, 1.6, 1.6, 1.6};
        double gear_ratio_[7];
        
        int sign_[7];
        double last_pos[7];

};
#endif // EYOU_JOINTS_NEW_H