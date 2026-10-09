#include "eyou_joints.h"
#include <unistd.h>

EyouJoints::EyouJoints(uint32_t* motor_ids, int* enable_motors){
    for (int i = 0; i < NUM_JOINTS; ++i) {
        motor_id_[i] = motor_ids[i];
        enable_motors_[i] = enable_motors[i];
    }
    isSync_ = true;
    type_ = isSync_ ? 0x01 : 0xFF;
}

EyouJoints::~EyouJoints() {
    // 清理资源
}

#ifdef ARM_ARCH
    void EyouJoints::canable_init()
{
    if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, DEVICE_INDEX, harmonic_Baudrate_1000))
    {
        std::cout << "[error]test open failed!" << std::endl;
    }
    else
    {
        std::cout << "[info]test open success!" << std::endl;
    }
}

void EyouJoints::canable_close()
{
    harmonic_freeDLL(DEVICE_INDEX);
}


void EyouJoints::set_pos(int index, int pos)
{
    huint8 data[4];
    data[0] = (unsigned)pos & 0x000000ff;
    data[1] = ((unsigned)pos & 0x0000ff00) >> 8;
    data[2] = ((unsigned)pos & 0x00ff0000) >> 16;
    data[3] = ((unsigned)pos & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX, 0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX, 0x80, data, 1);
    }
}
void EyouJoints::set_pos(int index, double pos_degrees)
{
    int pos = (int)(pos_degrees/PULSE2DEG);
    huint8 data[4];
    data[0] = (unsigned)pos & 0x000000ff;
    data[1] = ((unsigned)pos & 0x0000ff00) >> 8;
    data[2] = ((unsigned)pos & 0x00ff0000) >> 16;
    data[3] = ((unsigned)pos & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX,0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX, 0x80, data, 1);
    }
}


void EyouJoints::set_velocity(int index, int vel)
{

    huint8 data[4];
    data[0] = (unsigned)vel & 0x000000ff;
    data[1] = ((unsigned)vel & 0x0000ff00) >> 8;
    data[2] = ((unsigned)vel & 0x00ff0000) >> 16;
    data[3] = ((unsigned)vel & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX,0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX,0x80, data, 1);
    }
}

void EyouJoints::set_velocity(int index, double vel_degree)
{
    int vel = (int)(vel_degree/PULSE2DEG);
    huint8 data[4];
    data[0] = (unsigned)vel & 0x000000ff;
    data[1] = ((unsigned)vel & 0x0000ff00) >> 8;
    data[2] = ((unsigned)vel & 0x00ff0000) >> 16;
    data[3] = ((unsigned)vel & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX, 0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX, 0x80, data, 1);
    }
}

void EyouJoints::set_torque(int index, int tor)
{
    huint8 data[4];
    data[0] = (unsigned)tor & 0x000000ff;
    data[1] = ((unsigned)tor & 0x0000ff00) >> 8;
    data[2] = ((unsigned)tor & 0x00ff0000) >> 16;
    data[3] = ((unsigned)tor & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX,0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX,0x80, data, 1);
    }
}
void EyouJoints::set_torque(int index, double tor_nm)
{
    int tor = (int)(tor_nm*1000.0/rated_torque_[index]);
    huint8 data[4];
    data[0] = (unsigned)tor & 0x000000ff;
    data[1] = ((unsigned)tor & 0x0000ff00) >> 8;
    data[2] = ((unsigned)tor & 0x00ff0000) >> 16;
    data[3] = ((unsigned)tor & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX, 0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX, 0x80, data, 1);
    }
}
void EyouJoints::all_motor_stop(int trials)
{
    for (int j = 0;j<trials;++j)
    {
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            if(!enable_motors_[i]) continue;
            //set_velocity(i, 0);
            stop_control(i);
        }
    }  
}

double EyouJoints::get_position(huint8 index, int* ret_ptr)
{
    hint32 curPos = 0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualPos(DEVICE_INDEX, id, &curPos);
    double pos_degrees = curPos*PULSE2DEG;
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout << "[test]getActualPos failed !" << std::endl;
        std::cout << "get_position fail ret: "<< ret <<"\n";
        return ret;
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return pos_degrees;//单位：
}
hint32 EyouJoints::get_position_pulse(huint8 index, int* ret_ptr)
{
    hint32 curPos = 0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualPos(DEVICE_INDEX,id, &curPos);
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout << "[test]getActualPos failed !" << std::endl;
        std::cout << "get_position_pulse fail ret: "<< ret <<"\n";
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return curPos;//单位：弧度
}

double EyouJoints::get_velocity(huint8 index, int* ret_ptr)
{
    hint32 curVel = 0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualVelocity(DEVICE_INDEX, id, &curVel);
    double rad_degrees = curVel*PULSE2DEG;
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout << "[test]getActualVelocity failed !" << std::endl;
        std::cout << "get_velocity fail ret: "<< ret <<"\n";
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return rad_degrees;//单位：rad/s
}


double EyouJoints::get_current(huint8 index, int* ret_ptr)
{
    hint16 curTor = 0;
    float Kt = rated_torque_[index]/1000.0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualTorque(DEVICE_INDEX,id, &curTor);
    if (HARMONIC_SUCCESS != ret) 
    {
        std::cout << "[test]getActualTorque failed !" << std::endl;
        return -1;
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return curTor*Kt;//单位Nm
}

void EyouJoints::stop_control(huint8 index)
{
    //todo:根据力矩和力矩常数计算电流,找厂家要力矩常数
    int id = motor_id_[index];
    harmonic_stopControl(DEVICE_INDEX,id);
}

int EyouJoints::set_operate_mode(harmonic_OperateMode mode)
{

    for (int i = 0;i<NUM_JOINTS;++i)
    {
        printf("enable_motors_[%d], %d\n",i, enable_motors_[i]);
        if (enable_motors_[i] == 0) 
            continue;
        huint32 RPDOMapped_value;
        if (mode == harmonic_OperateMode_CyclicSyncPosition)
            RPDOMapped_value = ((0x607A << 16) + 0x020);
        else if (mode == harmonic_OperateMode_CyclicSyncVelocity)
            RPDOMapped_value = ((0x60FF << 16) + 0x020);
        else if (mode == harmonic_OperateMode_CyclicSyncTorque)
            RPDOMapped_value = ((0x6071 << 16) + 0x010);
        else
            RPDOMapped_value = ((0x6071 << 16) + 0x010);
        // 设置伺服驱动器的操作模式为循环同步位置模式
        huint8 id = motor_id_[i];
        if (HARMONIC_SUCCESS != harmonic_setOperateMode(DEVICE_INDEX,id, mode))
        {
            std::cout << "[test]setOperateMode failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置插值时间周期值
        if (HARMONIC_SUCCESS != harmonic_setInterpolationTimePeriodValue(DEVICE_INDEX,id, ITPV))
        {
            std::cout << "[test]setInterpolationTimePeriodValue failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置同步计数器值为0
        if (HARMONIC_SUCCESS != harmonic_setSyncCounter(DEVICE_INDEX,id, 0))
        {
            std::cout << "[test]setSyncCounter failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置RPDO（接收过程数据对象）的通信标识符
        if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(DEVICE_INDEX,id, 0, (0x80 << 24) + 0x200 + id))
        {
            std::cout << "[test]setRPDOCobId failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置RPDO的最大映射对象数量为0
        if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(DEVICE_INDEX,id, 0, 0))
        {
            std::cout << "[test]setRPDOMaxMappedCount failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置RPDO的传输类型
        if (HARMONIC_SUCCESS != harmonic_setRPDOTransmitType(DEVICE_INDEX,id, 0, type_))
        {
            std::cout << "[test]setRPDOTransmitType failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }
        
        // 设置RPDO的映射对象
        if (HARMONIC_SUCCESS != harmonic_setRPDOMapped(DEVICE_INDEX,id, 0, 0, RPDOMapped_value))
        {
            std::cout << "[test]setRPDOMapped failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 再次设置RPDO的最大映射对象数量为1
        if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(DEVICE_INDEX,id, 0, 1))
        {
            std::cout << "[test]setRPDOMaxMappedCount failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置节点状态为重置节点
        if (HARMONIC_SUCCESS != harmonic_setNodeState(DEVICE_INDEX,id, harmonic_NMTState_Reset_Node))
        {
            std::cout << "[test]setNodeState failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 等待50毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 设置节点状态为启动节点
        if (HARMONIC_SUCCESS != harmonic_setNodeState(DEVICE_INDEX,id, harmonic_NMTState_Start_Node))
        {
            std::cout << "[test]setNodeState failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 等待50毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 设置RPDO的通信标识符
        if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(DEVICE_INDEX, id, 0, 0x200 + id))
        {
            std::cout << "[test]setRPDOCobId failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置控制字为0x06
        if (HARMONIC_SUCCESS != harmonic_setControlword(DEVICE_INDEX,id, 0x06))
        {
            std::cout << "[test]setControlword failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置控制字为0x07
        if (HARMONIC_SUCCESS != harmonic_setControlword(DEVICE_INDEX,id, 0x07))
        {
            std::cout << "[test]setControlword failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置控制字为0x0F
        if (HARMONIC_SUCCESS != harmonic_setControlword(DEVICE_INDEX,id, 0x0F))
        {
            std::cout << "[test]setControlword failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }
    }
    return 0;
}

#else
static void printHexArray(bool isSend, const unsigned char *data, size_t length)
    {
        if (isSend)
            std::cout << "send:";
        else
            std::cout << "receive:";
        for (size_t i = 0; i < length; ++i)
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
        std::cout << std::endl;
    }

static void sendCallback(const harmonic_CanMsg &msg)
{
    printHexArray(true, msg.data, msg.len);
}
static void receiveCallback(const harmonic_CanMsg &msg)
    {
        printHexArray(false, msg.data, msg.len);
    }



void EyouJoints::canable_init()
{
    if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable,DEVICE_INDEX, harmonic_Baudrate_1000))
    {
        std::cout << "[error]test open failed!" << std::endl;
    }
    else
    {
        std::cout << "[info]test open success!" << std::endl;
    }
}

void EyouJoints::canable_close()
{
    harmonic_freeDLL(DEVICE_INDEX);
}


void EyouJoints::set_pos(int index, int pos)
{
    huint8 data[4];
    data[0] = (unsigned)pos & 0x000000ff;
    data[1] = ((unsigned)pos & 0x0000ff00) >> 8;
    data[2] = ((unsigned)pos & 0x00ff0000) >> 16;
    data[3] = ((unsigned)pos & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX,0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX,0x80, data, 1);
    }
}
void EyouJoints::set_pos(int index, double pos_degree)
{
    int pos = (int)(pos_degree/PULSE2DEG);
    huint8 data[4];
    data[0] = (unsigned)pos & 0x000000ff;
    data[1] = ((unsigned)pos & 0x0000ff00) >> 8;
    data[2] = ((unsigned)pos & 0x00ff0000) >> 16;
    data[3] = ((unsigned)pos & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX,0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX,0x80, data, 1);
    }
}
void EyouJoints::set_velocity(int index, int vel)
{

    huint8 data[4];
    data[0] = (unsigned)vel & 0x000000ff;
    data[1] = ((unsigned)vel & 0x0000ff00) >> 8;
    data[2] = ((unsigned)vel & 0x00ff0000) >> 16;
    data[3] = ((unsigned)vel & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX,0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX,0x80, data, 1);
    }
}

void EyouJoints::set_velocity(int index, double vel_degree)
{
    int vel = (int)(vel_degree/PULSE2DEG);
    huint8 data[4];
    data[0] = (unsigned)vel & 0x000000ff;
    data[1] = ((unsigned)vel & 0x0000ff00) >> 8;
    data[2] = ((unsigned)vel & 0x00ff0000) >> 16;
    data[3] = ((unsigned)vel & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX, 0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX,0x80, data, 1);
    }
}

void EyouJoints::set_torque(int index, int tor)
{
    huint8 data[4];
    data[0] = (unsigned)tor & 0x000000ff;
    data[1] = ((unsigned)tor & 0x0000ff00) >> 8;
    data[2] = ((unsigned)tor & 0x00ff0000) >> 16;
    data[3] = ((unsigned)tor & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX,0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX, 0x80, data, 1);
    }
}
void EyouJoints::set_torque(int index, double tor_nm)
{
    int tor = (int)(tor_nm*1000.0/rated_torque_[index]);
    huint8 data[4];
    data[0] = (unsigned)tor & 0x000000ff;
    data[1] = ((unsigned)tor & 0x0000ff00) >> 8;
    data[2] = ((unsigned)tor & 0x00ff0000) >> 16;
    data[3] = ((unsigned)tor & 0xff000000) >> 24;
    int id = motor_id_[index];
    harmonic_writeCanData(DEVICE_INDEX, 0x200 + id, data, 4);
    if (isSync_)
    {
        huint8 data[1] = {0};
        harmonic_writeCanData(DEVICE_INDEX,0x80, data, 1);
    }
}
void EyouJoints::all_motor_stop(int trials)
{
    for (int j = 0;j<trials;++j)
    {
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            if(!enable_motors_[i]) continue;
            //set_velocity(i, 0);
            stop_control(i);
        }
    }  
}

double EyouJoints::get_position(huint8 index, int* ret_ptr)
{
    hint32 curPos = 0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualPos(DEVICE_INDEX, id, &curPos);
    double pos_degrees = curPos*PULSE2DEG;
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout << "Motor " << (int)index << " get_position failed! Ret: " << ret << std::endl;
        return ret;
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return pos_degrees;//单位：弧度
}
hint32 EyouJoints::get_position_pulse(huint8 index, int* ret_ptr)
{
    hint32 curPos = 0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualPos(DEVICE_INDEX,id, &curPos);
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout << "Motor " << (int)index << " get_position_pulse failed! Ret: " << ret << std::endl;
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return curPos;//单位：弧度
}
double EyouJoints::get_velocity(huint8 index, int* ret_ptr)
{
    hint32 curVel = 0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualVelocity(DEVICE_INDEX, id, &curVel);
    double rad_degrees = curVel*PULSE2DEG;
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout << "Motor " << (int)index << " get_velocity failed! Ret: " << ret << std::endl;
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return rad_degrees;//单位：rad/s
}


double EyouJoints::get_current(huint8 index, int* ret_ptr)
{
    hint16 curTor = 0;
    float Kt = rated_torque_[index]/1000.0;
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getActualTorque(DEVICE_INDEX,id, &curTor);
    if (HARMONIC_SUCCESS != ret) 
    {
        std::cout << "Motor " << (int)index << " get_current failed! Ret: " << ret << std::endl;
        return -1;
    }
    if (ret_ptr != nullptr){
        *ret_ptr = ret;
    }
    return curTor*Kt;//单位Nm
}
void EyouJoints::stop_control(huint8 index)
{
    //todo:根据力矩和力矩常数计算电流,找厂家要力矩常数
    int id = motor_id_[index];
    harmonic_stopControl(DEVICE_INDEX,id);
}
int EyouJoints::set_operate_mode(harmonic_OperateMode mode)
{
    for (int i = 0;i<NUM_JOINTS;++i)
    {
        if (enable_motors_[i] == 0) 
            continue;
        huint32 RPDOMapped_value;
        if (mode == harmonic_OperateMode_CyclicSyncPosition)
            RPDOMapped_value = ((0x607A << 16) + 0x020);
        else if (mode == harmonic_OperateMode_CyclicSyncVelocity)
            RPDOMapped_value = ((0x60FF << 16) + 0x020);
        else if (mode == harmonic_OperateMode_CyclicSyncTorque)
            RPDOMapped_value = ((0x6071 << 16) + 0x010);
        else
            RPDOMapped_value = ((0x6071 << 16) + 0x010);
        // 设置伺服驱动器的操作模式为循环同步位置模式
        huint8 id = motor_id_[i];

        int stop_control_ret = harmonic_stopControl(DEVICE_INDEX,id);
        // std::cout << "motor " << i << " stop_control ret: " << stop_control_ret << std::endl;

        if (HARMONIC_SUCCESS != harmonic_setOperateMode(DEVICE_INDEX,id, mode))
        {
            std::cout << "[test]setOperateMode failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置插值时间周期值
        if (HARMONIC_SUCCESS != harmonic_setInterpolationTimePeriodValue(DEVICE_INDEX,id, ITPV))
        {
            std::cout << "[test]setInterpolationTimePeriodValue failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置同步计数器值为0
        if (HARMONIC_SUCCESS != harmonic_setSyncCounter(DEVICE_INDEX,id, 0))
        {
            std::cout << "[test]setSyncCounter failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置RPDO（接收过程数据对象）的通信标识符
        if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(DEVICE_INDEX,id, 0, (0x80 << 24) + 0x200 + id))
        {
            std::cout << "[test]setRPDOCobId failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置RPDO的最大映射对象数量为0
        if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(DEVICE_INDEX,id, 0, 0))
        {
            std::cout << "[test]setRPDOMaxMappedCount failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置RPDO的传输类型
        if (HARMONIC_SUCCESS != harmonic_setRPDOTransmitType(DEVICE_INDEX,id, 0, type_))
        {
            std::cout << "[test]setRPDOTransmitType failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }
        
        // 设置RPDO的映射对象
        if (HARMONIC_SUCCESS != harmonic_setRPDOMapped(DEVICE_INDEX,id, 0, 0, RPDOMapped_value))
        {
            std::cout << "[test]setRPDOMapped failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 再次设置RPDO的最大映射对象数量为1
        if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(DEVICE_INDEX,id, 0, 1))
        {
            std::cout << "[test]setRPDOMaxMappedCount failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置节点状态为重置节点
        if (HARMONIC_SUCCESS != harmonic_setNodeState(DEVICE_INDEX,id, harmonic_NMTState_Reset_Node))
        {
            std::cout << "[test]setNodeState failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 等待50毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 设置节点状态为启动节点
        if (HARMONIC_SUCCESS != harmonic_setNodeState(DEVICE_INDEX,id, harmonic_NMTState_Start_Node))
        {
            std::cout << "[test]setNodeState failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 等待50毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 设置RPDO的通信标识符
        if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(DEVICE_INDEX,id, 0, 0x200 + id))
        {
            std::cout << "[test]setRPDOCobId failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置控制字为0x06
        if (HARMONIC_SUCCESS != harmonic_setControlword(DEVICE_INDEX,id, 0x06))
        {
            std::cout << "[test]setControlword failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置控制字为0x07
        if (HARMONIC_SUCCESS != harmonic_setControlword(DEVICE_INDEX,id, 0x07))
        {
            std::cout << "[test]setControlword failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }

        // 设置控制字为0x0F
        if (HARMONIC_SUCCESS != harmonic_setControlword(DEVICE_INDEX,id, 0x0F))
        {
            std::cout << "[test]setControlword failed !" << std::endl;
            harmonic_freeDLL(DEVICE_INDEX);
            return -1;
        }
    }
    return 0;
}

int EyouJoints::get_operate_mode(huint8 index, harmonic_OperateMode* mode_ptr)
{
    int ret = 0;
    int id = motor_id_[index];
    ret = harmonic_getOperateMode(DEVICE_INDEX, id, mode_ptr);
    if (HARMONIC_SUCCESS != ret)
    {
        std::cout << "Motor " << (int)index << " getOperateMode failed! Ret: " << ret << std::endl;
        return ret;
    }
    return 0;
}

#endif