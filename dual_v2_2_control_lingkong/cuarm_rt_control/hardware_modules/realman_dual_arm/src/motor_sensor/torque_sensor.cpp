#include "torque_sensor.h"

TorqueSensor::TorqueSensor(const unsigned int sensor_id, const int channels,const USB_CAN_DEVICE device_type, const int sign) {
    device_type_ = device_type;
    sensor_id_ = sensor_id;
    can_channel_ = channels;
    sign_ = sign;
}

TorqueSensor::~TorqueSensor() {
    // 清理资源
}


double TorqueSensor::read_sensor() {

    
    unsigned char send_data[8];
    unsigned char receive_data[8];
    unsigned int receive_id = 0;
    for (int i = 0; i < 8; ++i)
        send_data[i] = 0x00;
    send_data[0] = 0x55;
    control_success_ = 0;
    return can_normal_send(device_type_, sensor_id_, send_data, can_channel_, can_channel_);
}

void TorqueSensor::decode_sensor_data(const unsigned char buffer[]) {
    int16_t data = ((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]);
    torque_data_ = data * 0.001f;
    
}
