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
#include "can_normal.h"
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <map>
#include <mutex>

class TorqueSensor {
public:


    TorqueSensor(const unsigned int sensor_id = 15, const int channels = 0, const USB_CAN_DEVICE device_type = EU_CANABLE, const int sign = 1);
    ~TorqueSensor();
    double read_sensor();
    unsigned int sensor_id_;
    int sign_;
    int control_success_ = 0;
    void decode_sensor_data(const unsigned char buffer[]);
    double torque_data_;
    USB_CAN_DEVICE device_type_;
private:
    
    
    int can_channel_;
    
};

#endif // EyoTorSenor_H