#include <iostream>
#include "eu_harmonic.h"
#include <thread>
#include <iomanip>
#include <csignal>
#include <math.h>
#define PULSE2DEG 0.00549316 
#define PULSE2RAD (2*M_PI/65536)
#define DEG2RAD (M_PI/180)
#define ITPV 10 //插补周期，单位：ms
#define CurtoTor 0.0847
#ifdef ARM_ARCH
    #define DEVICE_INDEX 2
#else
    #define DEVICE_INDEX 0
#endif
class EyouJoints{
    public:
        const int NUM_JOINTS = 14;
        EyouJoints(uint32_t* motor_ids, int* enable_motors);
        ~EyouJoints();
        void canable_init();
        void canable_close();
        void set_pos(int index, int pos);
        void set_pos(int index, double pos_degree);
        void set_velocity(int index, int vel);
        void set_velocity(int index, double vel_degree);
        void set_torque(int index, int tor);
        void set_torque(int index, double tor_nm);

        void all_motor_stop(int trials=5);
        double get_position(huint8 index, int* ret_ptr=nullptr);
        double get_velocity(huint8 index, int* ret_ptr=nullptr);
        double get_current(huint8 index, int* ret_ptr=nullptr); 
        hint32 get_position_pulse(huint8 index, int* ret_ptr=nullptr);
        int set_operate_mode(harmonic_OperateMode mode); 
        int get_operate_mode(huint8 index, harmonic_OperateMode* mode_ptr);  
        
        void stop_control(huint8 index);
    private:
        bool isSync_;
        huint8 type_;
        uint32_t motor_id_[14];
        bool enable_motors_[14];
        int gear_ration_[14] = {121, 121, 101, 101, 101, 101, 101, 121, 121, 101, 101, 101, 101, 101};
        float rated_torque_[14] = {78.0, 78.0, 8.6, 8.6, 1.6, 1.6, 1.6, 78.0, 78.0, 8.6, 8.6, 1.6, 1.6, 1.6};
        void setSendDataCallBack();
        void setReceiveDataCallBack();
};
