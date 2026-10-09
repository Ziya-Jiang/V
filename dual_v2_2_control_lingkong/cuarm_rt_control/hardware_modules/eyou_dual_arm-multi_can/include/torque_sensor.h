#ifndef TorSenor_H
#define TorSenor_H

#include <iostream>
#include <memory>
#include <ctime>
#include <cmath>
#include <time.h>
#include <thread>
#include <signal.h>
#include <fstream>
#include "usb_to_can.h"
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <map>
#include <mutex>
enum CanDeviceType {
    DEVICE_CANABLE = 0,  // 第一种USB转CAN设备
    DEVICE_USBCANII = 1    // 第二种USB转CAN设备
};
class TorSensor {
public:


    TorSensor(CanDeviceType device_type = DEVICE_USBCANII, int arm_index = 0,int num_sensors = 0, 
                unsigned int* sensor_ids = nullptr, int channels = 0);
    ~TorSensor();

    int read_sensor(int sensor_index);
    float get_torque_data(int sensor_index) const;
    static void staticReceiveCallback(int dev_index, const harmonic_CanMsg* msg);
private:
    CanDeviceType device_type_;
    void decode_sensor_data(const BYTE buffer[], int sensor_index);
    int num_sensors_;
    int arm_index_;
    unsigned int sensor_ids_[7];
    int can_channel_;
    float torque_data_[7];
    static std::map<int, TorSensor*> instance_map_;  // arm_index到实例指针的映射
    static std::mutex map_mutex_;
};

#endif // EyoTorSenor_H