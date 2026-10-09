#include "torque_sensor.h"


static int id_to_index(uint32_t id) {
    switch(id) {
        case 0x15: return 0;
        case 0x16: return 1;
        case 0x17: return 2;
        case 0x18: return 3;
        case 0x19: return 4;
        case 0x20: return 5;
        case 0x21: return 6;
        case 0x22: return 7;
        case 0x23: return 8;
        case 0x24: return 9;
        case 0x25: return 10;
        case 0x26: return 11;
        case 0x27: return 12;
        case 0x28: return 13;
        default: return -1;  // 无效ID
    }
}

EyoTorSenor::EyoTorSenor(uint32_t sensor_ids[]) {
    for (int i = 0; i < NUM_SENSORS; ++i) {
        sensor_ids_[i] = sensor_ids[i];
        torque_data_[i] = 0.0f;
        if (sensor_ids_[i] >=0x22) can_channel_[i] = 1;
        else can_channel_[i] = 0;
    }
}

EyoTorSenor::~EyoTorSenor() {
    // 清理资源
}


int EyoTorSenor::read_sensor(int sensor_index) {

    if (sensor_index < 0 || sensor_index >= NUM_SENSORS) {
        std::cerr << "Invalid sensor index: " << sensor_index << std::endl;
        return -2;
    }

    BYTE send_data[8];
    BYTE receive_data[8];
    unsigned int receive_id = 0;
    for (int i = 0; i < 8; ++i)
        send_data[i] = 0x00;
    send_data[0] = 0x55;

    serial_can_send(sensor_ids_[sensor_index], send_data, 0, can_channel_[sensor_index]);
    serial_can_receive(receive_data, &receive_id, 0, can_channel_[sensor_index]);
    int received_index = id_to_index(receive_id);
    decode_sensor_data(receive_data, received_index);
    return 0;
}

void EyoTorSenor::decode_sensor_data(const BYTE buffer[], int sensor_index) {
    int16_t data = ((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]);
    torque_data_[sensor_index] = data * 0.001f;
    
}

float EyoTorSenor::get_torque_data(int sensor_index) const {
    if (sensor_index < 0 || sensor_index >= NUM_SENSORS) {
        std::cerr << "Invalid sensor index: " << sensor_index << std::endl;
        return 0.0f;
    }
    return torque_data_[sensor_index];
}
