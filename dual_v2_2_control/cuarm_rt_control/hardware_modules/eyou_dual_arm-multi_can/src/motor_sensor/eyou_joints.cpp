#include "eyou_joints.h"

std::map<int, EyouJoints*> EyouJoints::instance_map_;
std::mutex EyouJoints::map_mutex_;
static bool flag_setcallback = false;
EyouJoints::EyouJoints(int arm_index, int num_joints, int can_channel, int use_pdo,
                         uint32_t* motor_ids, int* enable_motors, double* gear_ratio,
                          int * sign, double *init_mdh_degrees)
{
    if(num_joints>7 | num_joints<0)
    {
        std::cout << "num_joints must be in [1, 7] range!" << std::endl;
        return;
    }
    can_channel_ = can_channel;
    num_joints_ = num_joints;
    arm_index_ = arm_index;
    use_pdo_ = use_pdo;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        instance_map_[arm_index_] = this;
        
    }
    for (int i = 0; i < num_joints_; ++i) {
        motor_id_[i] = motor_ids[i];
        enable_motors_[i] = enable_motors[i];
        init_mdh_degrees_[i] = init_mdh_degrees[i];
        gear_ratio_[i] = gear_ratio[i];
        sign_[i] = sign[i];
        act_pos_degrees[i] = 0.0;
        act_vel_dps[i] = 0.0;
    }
    isSync_ = true;
    type_ = isSync_ ? 0x01 : 0xFF;
}

EyouJoints::~EyouJoints() {
    // 清理资源
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = instance_map_.find(arm_index_);
    if (it != instance_map_.end()) {
        instance_map_.erase(it);
        std::cout << "instance arm:" << arm_index_ << " erase" << std::endl;
    }
}

void EyouJoints::all_motor_stop(int trial_times)
{
    for (int j = 0; j < num_joints_; j++)
    {
        motor_stop(j, trial_times);
    }
}

void EyouJoints::motor_stop(huint8 index, int trial_times){
    if(!enable_motors_[index]) return;

    for (int i = 0; i<trial_times; ++i)
    {   
        switch (operate_mode_[index])
        {
            case harmonic_OperateMode::harmonic_OperateMode_CyclicSyncPosition:
            {
                int ret;
                double pos = get_position(index, &ret);
                if (ret == HARMONIC_SUCCESS){
                    set_pos(index, pos);
                }else{
                    printf("motor_stop(): Trial %d, Joint %d failed get position\n", i, index);
                }
                break;
            }
        
            case harmonic_OperateMode::harmonic_OperateMode_CyclicSyncVelocity:
                set_velocity(index, 0.0);
                break;
        
            case harmonic_OperateMode::harmonic_OperateMode_CyclicSyncTorque:
                set_torque(index, 0.0);
                break;
                
            default:
                break;
        }
    } 
    hard_stop(index);
}

void EyouJoints::hard_stop_all(int trial_times)
{ 
    for (int i = 0; i<trial_times; ++i)
    {
        for (int j = 0; j < num_joints_; j++)
        {
            if(!enable_motors_[j]) continue;
            hard_stop(j);
        }
    }  
}

//Set 0 torque to motor and lock the brake when velocity reaches 0
void EyouJoints::hard_stop(huint8 index)
{
    int id = motor_id_[index];
    harmonic_stopControl(can_channel_,id);
}

int EyouJoints::check_position_jump(huint8 index)
{
    std::lock_guard<std::mutex> lock(check_mutex_);
    if (std::fabs(act_pos_degrees[index]-last_pos[index])>50.0)
    {
        std::cerr << "[ERROR] joint ID "<< motor_id_[index] <<" jump happens, " <<" error "
                    << act_pos_degrees[index]-last_pos[index] <<std::endl;
        all_motor_stop();
        return -1;
    }
    return 0;
}
int EyouJoints::check_position_following(huint8 index, double target_pos)
{
    std::lock_guard<std::mutex> lock(check_mutex_);
    if (std::fabs(target_pos - act_pos_degrees[index]) > 1000.0)
    {
        std::cerr << "[ERROR] Joint ID "<< motor_id_[index] <<" target pos "<< target_pos <<" actual pos "
                    << act_pos_degrees[index] <<" not match"<<std::endl;
        all_motor_stop();
        return -1;
    }
    return 0;

}
int EyouJoints::check_velocity_following(huint8 index, double target_vel)
{
    std::lock_guard<std::mutex> lock(check_mutex_);
    if (std::fabs(target_vel - act_vel_dps[index]) > 5000.0)
    {
        std::cerr << "[ERROR] Joint ID "<< motor_id_[index] <<" target vel "<< target_vel <<" actual vel "
                    << act_vel_dps[index] <<" not match"<<std::endl;
        all_motor_stop();
        return -1;
    }
    return 0;
    
}
int EyouJoints::check_torque_following(huint8 index, double target_tor)
{
    std::lock_guard<std::mutex> lock(check_mutex_);
    if (std::fabs(target_tor - act_tor_nm[index])> 5.0)
    {
        std::cerr << "[ERROR] Joint  "<< motor_id_[index] <<" target tor "<< target_tor <<" actual tor "
                    << act_vel_dps[index] <<" not match"<<std::endl;
        all_motor_stop();
        return -1;
    }
    return 0;
    
}
double EyouJoints::mdh_to_joints(int index, const double mdh_degrees)
{
    double joints_degrees;
    joints_degrees = init_mdh_degrees_[index] + sign_[index] *mdh_degrees * gear_ratio_[index];
    return joints_degrees;
}

double EyouJoints::joints_to_mdh(int index, const double joints_degrees)
{
    double mdh_degrees;
    mdh_degrees = (sign_[index] * (joints_degrees - init_mdh_degrees_[index]))/gear_ratio_[index];
    return mdh_degrees;
}

int EyouJoints::set_mdhpos(int index, double mdhpos_degrees)
{
    double joint_degrees = mdh_to_joints(index, mdhpos_degrees);
    if(set_pos(index, joint_degrees) == -1)
        return -1;
    return 0;
}

int EyouJoints::set_pos(int index, double pos_degrees)
{
    // if (use_pdo_)
    // {
    //     std::lock_guard<std::mutex> lock(map_mutex_);
    //     if (check_position_following(index, pos_degrees)==-1)
    //     {
    //         return -1;
    //     }else if(check_position_jump(index) == -1)
    //     {
    //         return -2;
    //     }
    // }
    int pos = (int)(pos_degrees/PULSE2DEG);
    huint8 data[4];
    data[0] = (unsigned)pos & 0x000000ff;
    data[1] = ((unsigned)pos & 0x0000ff00) >> 8;
    data[2] = ((unsigned)pos & 0x00ff0000) >> 16;
    data[3] = ((unsigned)pos & 0xff000000) >> 24;
    int id = motor_id_[index];
    //std::cout << "index "<< index <<" motor_id_ "<< motor_id_[index] <<std::endl;
    harmonic_writeCanData(can_channel_,0x200 + id, data, 4);

    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(can_channel_, 0x80, data, 1);
    }
    return 0;
}

int EyouJoints::set_velocity(int index, double vel_degree)
{
    if (use_pdo_)
    {
        if (check_velocity_following(index, vel_degree)==-1)
        {
            return -1;
        }else if(check_position_jump(index) == -1)
        {
            return -2;
        }
    }
    int vel = (int)(vel_degree/PULSE2DEG);
    huint8 data[4];
    data[0] = (unsigned)vel & 0x000000ff;
    data[1] = ((unsigned)vel & 0x0000ff00) >> 8;
    data[2] = ((unsigned)vel & 0x00ff0000) >> 16;
    data[3] = ((unsigned)vel & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(can_channel_, 0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(can_channel_, 0x80, data, 1);
    }
    return 0;
}

int EyouJoints::set_torque(int index, double tor_nm)
{
    if (use_pdo_)
    {
        if (check_torque_following(index, tor_nm)==-1)
        {
            return -1;
        }else if(check_position_jump(index) == -1)
        {
            return -2;
        }
    }
    int tor = (int)(tor_nm*1000.0/rated_torque_[index]);
    huint8 data[4];
    data[0] = (unsigned)tor & 0x000000ff;
    data[1] = ((unsigned)tor & 0x0000ff00) >> 8;
    data[2] = ((unsigned)tor & 0x00ff0000) >> 16;
    data[3] = ((unsigned)tor & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(can_channel_, 0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(can_channel_, 0x80, data, 1);
    }
    return 0;
}

double EyouJoints::get_position(huint8 index, int* ret_ptr)
{
    if (use_pdo_)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (ret_ptr != nullptr) *ret_ptr = 0;
        return act_pos_degrees[index];
    }
    // std::cout<< "index "<< (int)index <<std::endl;
    hint32 curPos = 0;
    int ret = 0;
    int id = motor_id_[index];
    
    ret = harmonic_getActualPos(can_channel_, id, &curPos, 1000);
    
    double pos_degrees = curPos*PULSE2DEG;
    //std::cout<< "index "<< (int)index << " POS_DEGREE   "<< pos_degrees<<std::endl;
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout <<"Joint "<< motor_id_[index] << "getActualPos failed !" << std::endl;
        std::cout << "get_position fail ret: "<< ret <<"\n";
    }

    if (ret_ptr != nullptr) *ret_ptr = ret;
    return pos_degrees;//单位：
    
}
double EyouJoints::get_mdhpos(huint8 index, int* ret_ptr)
{
    double joint_degrees = get_position(index, ret_ptr);
    //std::cout << "index "<< (int)index << " " << joint_degrees << std::endl;
    return joints_to_mdh(index, joint_degrees);
}

double EyouJoints::get_velocity(huint8 index, int* ret_ptr)
{
    if (use_pdo_)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (ret_ptr != nullptr) *ret_ptr = 0;
        return act_vel_dps[index];//单位：rad/s
    }
    hint32 curVel = 0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualVelocity(can_channel_, id, &curVel);
    double vel_degrees = curVel*PULSE2DEG;
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout <<"Joint "<< motor_id_[index] << "[test]getActualVelocity failed !" << std::endl;
        std::cout << "get_velocity fail ret: "<< ret <<"\n";
    }

    if (ret_ptr != nullptr) *ret_ptr = ret;
    return vel_degrees;
}


double EyouJoints::get_current(huint8 index, int* ret_ptr)
{
    if (use_pdo_)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (ret_ptr != nullptr) *ret_ptr = 0;
        return act_tor_nm[index];//单位Nm
    }
    hint16 curTor = 0;
    float Kt = rated_torque_[index]/1000.0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualTorque(can_channel_,id, &curTor);
    if (HARMONIC_SUCCESS != ret) 
    {
        std::cout <<"Joint "<< motor_id_[index] << "[test]getActualTorque failed !" << std::endl;
    }

    if (ret_ptr != nullptr) *ret_ptr = ret;
    return curTor*Kt;//单位Nm
}

int EyouJoints::set_pdo(int autoReportMs)
{
    for (int i = 0;i<num_joints_;++i)
    {
        if (enable_motors_[i] == 0) 
            continue;
        for (int index = 0;index<2;++index)
        {
            harmonic_setTPDOCobId(can_channel_, motor_id_[i], index, (0x80 << 24) + 0x180 + index * 0x100 + motor_id_[i]);//无效tpdo 0x8000181
            harmonic_setTPDOMaxMappedCount(can_channel_, motor_id_[i], index, 0);
            harmonic_setTPDOTransmitType(can_channel_, motor_id_[i], index, 0xFF);//EventTimer清零自动上报
            harmonic_setTPDOInhibitTime(can_channel_, motor_id_[i], index, 0);
            harmonic_setTPDOEventTimer(can_channel_, motor_id_[i], index, autoReportMs);
            harmonic_setTPDOSYNCStartValue(can_channel_, motor_id_[i], index, 0);
        }
        //无效tpdo
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    }
    for (int i = 0;i<num_joints_;++i)
    {
        if (enable_motors_[i] == 0) 
            continue;
        for (int index = 0;index<2;++index)
        {
            if (index == 0)
            {
                harmonic_setTPDOMapped(can_channel_, motor_id_[i], index, 0, (0x6064 << 16) + 0x20); // 实际位置32bit
                harmonic_setTPDOMapped(can_channel_, motor_id_[i], index, 1, (0x606C << 16) + 0x20); // 实际速度32bit
                harmonic_setTPDOMaxMappedCount(can_channel_, motor_id_[i], index, 2);
                harmonic_setTPDOCobId(can_channel_, motor_id_[i], index, 0x180 + motor_id_[i]); // TPDO1是0x180
            }else{
                harmonic_setTPDOMapped(can_channel_, motor_id_[i], index, 0, (0x6077 << 16) + 0x10); // 实际扭矩 16bit
                harmonic_setTPDOMaxMappedCount(can_channel_, motor_id_[i], index, 1);
                harmonic_setTPDOCobId(can_channel_, motor_id_[i], index, 0x280 + motor_id_[i]); // TPDO2是0x280
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
}

void EyouJoints::fix_initial_position(){
    //Fix initial position one circle issue
    for (int i = 0;i<num_joints_;++i){
        if (enable_motors_[i] == 0) 
            continue;
        int retry_count = 3;
        for (int times = 0; times < retry_count; ++times)
        {
            zero_mdh_pos[i] = get_mdhpos(i);
            // std::cout << "arm["<<arm_index_<<"] joint["<<i<<"] zero pos: "<<zero_mdh_pos[i]<<std::endl;
        }
        if ((zero_mdh_pos[i] - joints_to_mdh(i, init_mdh_degrees_[i])) >=180.0)
        {
            std::cout << "arm " << arm_index_ << "motor " << i <<" lack one circle"<< std::endl;
            double pre_add = init_mdh_degrees_[i];
            init_mdh_degrees_[i] = init_mdh_degrees_[i] + 360 * gear_ratio_[i] * sign_[i];
            std::cout <<"brefore add: "<< pre_add <<", after add: "<< init_mdh_degrees_[i] <<std::endl;
        }else if ((zero_mdh_pos[i] - joints_to_mdh(i, init_mdh_degrees_[i])) <=-180.0)
        {
            std::cout << "arm " << arm_index_ << "motor " << i <<" add one circle"<< std::endl;
            double pre_add = init_mdh_degrees_[i];
            init_mdh_degrees_[i] = init_mdh_degrees_[i] - 360 * gear_ratio_[i] * sign_[i];
            std::cout <<"brefore add: "<< pre_add <<", after add: "<< init_mdh_degrees_[i] <<std::endl;
        }
        for (int times = 0; times < retry_count; ++times)
        {
            zero_mdh_pos[i] = get_mdhpos(i);
            // std::cout << "arm["<<arm_index_<<"] joint["<<i<<"] zero pos: "<<zero_mdh_pos[i]<<std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int EyouJoints::init_joints(int autoReportMs){
    if (use_pdo_)
        EyouJoints::set_pdo(autoReportMs);
    fix_initial_position();
    return 0;
}

int EyouJoints::init_joints(harmonic_OperateMode mode, int autoReportMs){
    if (use_pdo_)
        EyouJoints::set_pdo(autoReportMs);

    set_operate_mode(mode);
    fix_initial_position();
    return 0;
}

int EyouJoints::set_operate_mode(harmonic_OperateMode mode)
{
    for (int i = 0;i<num_joints_;++i){
        set_operate_mode(i, mode);
    }
    return 0;
}

int EyouJoints::set_operate_mode(huint8 index, harmonic_OperateMode mode)
{
    if (enable_motors_[index] == 0) return 0;
    
    int WaitTime = 1000;
    int maximum_retry = 20;
    huint32 RPDOMapped_value;
    if (mode == harmonic_OperateMode_CyclicSyncPosition)
        RPDOMapped_value = ((0x607A << 16) + 0x020);
    else if (mode == harmonic_OperateMode_CyclicSyncVelocity)
        RPDOMapped_value = ((0x60FF << 16) + 0x020);
    else if (mode == harmonic_OperateMode_CyclicSyncTorque)
        RPDOMapped_value = ((0x6071 << 16) + 0x010);
    else
        RPDOMapped_value = ((0x607A << 16) + 0x020);
        
    int retry_count = 0;
    bool success = false;
    int id = motor_id_[index];
    operate_mode_[index] = mode;

    //Stop motor first
    motor_stop(index, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //Wait for motor to stop

    // Set operate mode
    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {

        if (HARMONIC_SUCCESS == harmonic_setOperateMode(can_channel_, id, mode))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "Joint " << id << "[test]setOperateMode failed" << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setInterpolationTimePeriodValue(can_channel_, id, ITPV, WaitTime))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setInterpolationTimePeriodValue failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setSyncCounter(can_channel_, id, 0, WaitTime))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setSyncCounter failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setRPDOCobId(can_channel_, id, 0, (0x80 << 24) + 0x200 + id, WaitTime))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setRPDOCobId failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS ==  harmonic_setRPDOMaxMappedCount(can_channel_, id, 0, 0, WaitTime))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setRPDOMaxMappedCount failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setRPDOTransmitType(can_channel_, id, 0, type_, WaitTime))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setRPDOTransmitType failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setRPDOMapped(can_channel_, id, 0, 0, RPDOMapped_value))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setRPDOMapped failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    retry_count = 0;
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setRPDOMaxMappedCount(can_channel_, id, 0, 1, WaitTime))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setRPDOMaxMappedCount failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    retry_count = 0;    
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setRPDOCobId(can_channel_, id, 0, 0x200 + id, WaitTime))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "setRPDOCobId failed,id:" << id << std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    harmonic_setNodeState(can_channel_, id, harmonic_NMTState_Reset_Node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    harmonic_setNodeState(can_channel_, id, harmonic_NMTState_Start_Node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    retry_count = 0;    
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setControlword(can_channel_, id, 0x06, 100))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "enable failed,id:" << id<< std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    retry_count = 0;    
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setControlword(can_channel_, id, 0x07, 100))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "enable failed,id:" << id<< std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    retry_count = 0;    
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setControlword(can_channel_, id, 0x0F, 100))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "enable failed,id:" << id<< std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    retry_count = 0;    
    success = false;
    while (retry_count < maximum_retry && !success)
    {
        if (HARMONIC_SUCCESS == harmonic_setControlword(can_channel_, id, 0x1F, 100))
        {
            success = true;
        }
        retry_count++;
        if (!success && retry_count >= maximum_retry)
        {
            std::cout << "enable failed,id:" << id<< std::endl;
            // harmonic_freeDLL(can_channel_);
            // return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return 0;
}

void EyouJoints::staticReceiveCallback(int dev_index, const harmonic_CanMsg* msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    const harmonic_CanMsg& received_msg = *msg;
    uint32_t cob_id = received_msg.cob_id;

    if (cob_id >= 0x180 && cob_id <= 0x180+16) {
        uint32_t motor_id = cob_id - 0x180;
        
        hint32 pos_raw = (received_msg.data[3] << 24) | 
                         (received_msg.data[2] << 16) | 
                         (received_msg.data[1] << 8) | 
                         received_msg.data[0];
                         
        hint32 vel_raw = (received_msg.data[7] << 24) | 
                         (received_msg.data[6] << 16) | 
                         (received_msg.data[5] << 8) | 
                         received_msg.data[4];
        
        double pos_deg = pos_raw * PULSE2DEG;
        double vel_dps = vel_raw * PULSE2DEG;
        
        // 查找对应的实例
        
        //std::cout<< "Motor ID: " << motor_id << ", Position: " << pos_deg << " degrees, Velocity: " << vel_dps << " dps" << std::endl;//打印的出来
        for (auto& pair : instance_map_) {
            EyouJoints* instance = pair.second;
            
            for (int i = 0; i < instance->num_joints_; i++) {
                if (instance->motor_id_[i] == motor_id) {
                    if (instance->first_flag[i] == 1)
                    {
                        instance->first_flag[i] = 0;
                        instance->last_pos[i] = instance->act_pos_degrees[i];
                    }else{
                        instance->act_pos_degrees[i] = pos_deg;
                        instance->act_vel_dps[i] = vel_dps;
                        instance->last_pos[i] = instance->act_pos_degrees[i];
                    }
                    return; 
                }
            }
        }
    }
    else if (cob_id >= 0x280 && cob_id <= 0x280+16)
    {
        uint32_t motor_id = cob_id - 0x280;
        hint16 tor_row = (received_msg.data[1] << 8) | 
                         received_msg.data[0];               
        for (auto& pair : instance_map_) {
            EyouJoints* instance = pair.second;
            
            for (int i = 0; i < instance->num_joints_; i++) {
                if (instance->motor_id_[i] == motor_id) {
                    instance->act_tor_nm[i] = tor_row * instance->rated_torque_[i] / 1000.0; // 转矩单位转换
                    return; 
                }
            }
        }
    }
}

