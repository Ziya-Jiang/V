#include "torque_sensor.h"
std::map<int, TorSensor*> TorSensor::instance_map_;
std::mutex TorSensor::map_mutex_;

static int id_to_index(uint32_t id) {
    switch(id) {
        case 0x15: return 0;
        case 0x16: return 1;
        case 0x17: return 2;
        case 0x18: return 3;
        case 0x19: return 4;
        case 0x20: return 5;
        case 0x21: return 6;

        case 0x22: return 0;
        case 0x23: return 1;
        case 0x24: return 2;
        case 0x25: return 3;
        case 0x26: return 4;
        case 0x27: return 5;
        case 0x28: return 6;
        default: return -1;  // 无效ID
    }
}

TorSensor::TorSensor(CanDeviceType device_type, int arm_index,
                    int num_sensors, unsigned int* sensor_ids, int channels) {
    num_sensors_ = num_sensors;
    device_type_ = device_type;
    std::cout << "device_type: " << device_type_ << std::endl;
    arm_index_ = arm_index;
    can_channel_ = channels;
    for (int i = 0; i < num_sensors_; ++i) {
        sensor_ids_[i] = sensor_ids[i];
        torque_data_[i] = 0.0f;
    }
    if (device_type_ == DEVICE_CANABLE) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        instance_map_[arm_index_] = this; 
    }

}

TorSensor::~TorSensor() {
    // 清理资源
    if (device_type_ == DEVICE_CANABLE) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        auto it = instance_map_.find(arm_index_);
        if (it != instance_map_.end()) {
            instance_map_.erase(it);
            std::cout << "instance arm:" << arm_index_ << " erase" << std::endl;
        }
    }
}


int TorSensor::read_sensor(int sensor_index) {

    switch (device_type_) {
        case DEVICE_CANABLE: {
            canable_CanMsg write_msg;
            write_msg.isExtended = false;
            write_msg.isRTR = false;
            write_msg.dlc = 8;
            write_msg.id = sensor_ids_[sensor_index];
            write_msg.data[0] = 0x55;
            for (int i = 1; i < 8; ++i) {
                write_msg.data[i] = 0x00;
            }
            
            int ret = canable_writeMessage(can_channel_, &write_msg);
            if (ret != CANABLE_SUCCESS) {
                std::cerr << "Failed to write CAN message for sensor index: " << sensor_index << std::endl;
                return -1;
            }
            return 0;
        }
        
        case DEVICE_USBCANII: {
            BYTE send_data[8];
            BYTE receive_data[8];
            unsigned int receive_id = 0;
            for (int i = 0; i < 8; ++i)
                send_data[i] = 0x00;
            send_data[0] = 0x55;
            serial_can_send(sensor_ids_[sensor_index], send_data, 0, can_channel_);
            serial_can_receive(receive_data, &receive_id, 0, can_channel_);
            int received_index = id_to_index(receive_id);
            decode_sensor_data(receive_data, received_index);
            return 0;
        }
        default:
            throw std::logic_error("Unknown device type");
    }
}

void TorSensor::decode_sensor_data(const BYTE buffer[], int sensor_index) {
    int16_t data = ((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]);
    torque_data_[sensor_index] = data * 0.001f;
    
}

float TorSensor::get_torque_data(int sensor_index) const {
    return torque_data_[sensor_index];
}

void TorSensor::staticReceiveCallback(int dev_index, const harmonic_CanMsg* msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    const harmonic_CanMsg& received_msg = *msg;
    uint32_t cob_id = received_msg.cob_id;

    if (cob_id >= 0x15 && cob_id <= 0x28) {
        uint32_t received_sensor_id = cob_id;
        
        int16_t torque_raw =  ((int16_t)received_msg.data[0] << 8) 
                            | ((int16_t)received_msg.data[1]);
                         
        double torque_nm = torque_raw * 0.001f;
        
        // 查找对应的实例
        
        //std::cout<< "Motor ID: " << motor_id << ", Position: " << pos_deg << " degrees, Velocity: " << vel_dps << " dps" << std::endl;//打印的出来
        for (auto& pair : instance_map_) {
            TorSensor* instance = pair.second;
            
            for (int i = 0; i < instance->num_sensors_; i++) {
                if (instance->sensor_ids_[i] == received_sensor_id) {
                    instance->torque_data_[i] = torque_nm;
                    return; 
                }
            }
        }
    }
}