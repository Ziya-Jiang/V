#ifndef EyoTorSenor_H
#define EyoTorSenor_H

#include <iostream>
#include <memory>
#include <ctime>
#include <cmath>
#include <time.h>
#include <thread>
#include <signal.h>
#include <fstream>
#include "serial_can.h"

#ifdef WIN32
    #include <winsock.h>
#else
    #include <unistd.h>
    #include <sys/time.h>
    #include <sys/select.h>
#endif

class EyoTorSenor {
public:
    static const int NUM_SENSORS = 14; // 机械臂上的传感器数量

    EyoTorSenor(uint32_t sensor_ids[]);
    ~EyoTorSenor();

    int read_sensor(int sensor_index);
    float get_torque_data(int sensor_index) const;

private:
    void decode_sensor_data(const BYTE buffer[], int sensor_index);

    std::ofstream dataFile_;
    static std::ofstream *fwrite_data_;
    uint32_t sensor_ids_[NUM_SENSORS];
    int can_channel_[14];
    float torque_data_[NUM_SENSORS];
};

#endif // EyoTorSenor_H