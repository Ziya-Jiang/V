#include "usb_to_can.h"

void canbox_init(const std::vector<int>& channels)
{
    if (channels.empty()) {
        std::cout << "[info]No CAN channels selected for initialization." << std::endl;
        return;
    }

    bool all_success = true;
    bool has_valid_channel = false;

    for (int channel : channels) {
        switch (channel) {
            case 0: 
                if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, 0, harmonic_Baudrate_1000)) {
                    std::cout << "[error]CAN channel 0  initialization failed!" << std::endl;
                    all_success = false;
                } else {
                    std::cout << "[info]CAN channel 0  initialization successful!\n" << std::endl;
                    has_valid_channel = true;
                }
                break;
            case 1:  
                if (HARMONIC_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, 1, harmonic_Baudrate_1000)) {
                    std::cout << "[error]CAN channel 1  initialization failed!" << std::endl;
                    all_success = false;
                } else {
                    std::cout << "[info]CAN channel 1  initialization successful!\n" << std::endl;
                    has_valid_channel = true;
                }
                break;
                
            case 2:  
                if (CANABLE_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, 2, harmonic_Baudrate_1000)) {
                    std::cout << "[error]CAN channel 2 initialization failed!" << std::endl;
                    all_success = false;
                } else {
                    std::cout << "[info]CAN channel 2 initialization successful!\n" << std::endl;
                    has_valid_channel = true;
                }
                break;
            case 3: 
                if (CANABLE_SUCCESS != harmonic_initDLL(harmonic_DeviceType_Canable, 3, harmonic_Baudrate_1000)) {
                    std::cout << "[error]CAN channel 3 initialization failed!" << std::endl;
                    all_success = false;
                } else {
                    std::cout << "[info]CAN channel 3  initialization successful!\n" << std::endl;
                    has_valid_channel = true;
                }
                break;    
            default:
                std::cout << "[error]Invalid CAN channel: " << channel 
                          << "! Only channels 0, 1, and 2 are supported." << std::endl;
                all_success = false;
                break;
        }
    }

    if (all_success && has_valid_channel) {
        std::cout << "[info]All selected CAN channels initialized successfully!\n" << std::endl;
    }
}

void canbox_close(const std::vector<int>& channels)
{
    if (channels.empty()) {
        std::cout << "[info]No CAN channels selected for closing." << std::endl;
        return;
    }

    for (int channel : channels) {
        switch (channel) {
            case 0:  // Left Arm
                harmonic_freeDLL(0);
                // canable_freeDLL(0);
                std::cout << "[info]CAN channel 0 (Left Arm) closed." << std::endl;
                break;
                
            case 1:  // Right Arm
                harmonic_freeDLL(1);
                std::cout << "[info]CAN channel 1 (Right Arm) closed." << std::endl;
                break;
                
            case 2:  // Torque Sensor
                harmonic_freeDLL(2);  
                std::cout << "[info]CAN channel 2 closed." << std::endl;
                break;
            case 3:  // Torque Sensor
                harmonic_freeDLL(3);  
                std::cout << "[info]CAN channel 3  closed." << std::endl;
                break;    
            default:
                std::cout << "[error]Invalid CAN channel: " << channel 
                          << "! Only channels 0, 1, and 2 are supported." << std::endl;
                break;
        }
    }
    
    std::cout << "[info]Selected CAN channels closed successfully.\n" << std::endl;
}

int serial_can_open(DWORD channels)
{
    const DWORD DEVICE_TYPE = VCI_USBCAN2;
    const DWORD DEVICE_ID = 0;  // 固定设备号为0

    //打开设备
    if (VCI_OpenDevice(DEVICE_TYPE, DEVICE_ID, 0) != 1) {
        printf("Open Device Failed\n");
        return -1;
    }

    // 初始化配置参数（保持原有设置）
    VCI_INIT_CONFIG vic = {
        .AccCode = 0x80000008,
        .AccMask = 0xFFFFFFFF,
        .Filter = 1,
        .Timing0 = 0x00,  // 1000Kbps
        .Timing1 = 0x14,
        .Mode = 0
    };

    // 直接处理两个通道
    for (DWORD ch = 0; ch < 2; ch++) {
        // 检查是否要初始化当前通道
        if (!(channels & (1 << ch))) continue;
        //初始化通道
        if (VCI_InitCAN(DEVICE_TYPE, DEVICE_ID, ch, &vic) != 1) {
            VCI_CloseDevice(DEVICE_TYPE, DEVICE_ID);
            printf("Init CAN%d failed\n", ch);
            return -2;
        }

        // 启动通道
        if (VCI_StartCAN(DEVICE_TYPE, DEVICE_ID, ch) != 1) {
            VCI_CloseDevice(DEVICE_TYPE, DEVICE_ID);
            printf("Start CAN%d failed\n", ch);
            return -3;
        }
        printf("CAN%d init & start success\n", ch);
    }
    
    return 0;
}



int serial_can_send(unsigned int can_id, const BYTE* data, DWORD device_id, DWORD m_can_id)
{
    VCI_CAN_OBJ tx_buffer;
    tx_buffer.ID = can_id;
    tx_buffer.SendType = 0;
    tx_buffer.RemoteFlag = 0;
    tx_buffer.ExternFlag = 0;
    tx_buffer.DataLen = 8;

    memcpy(tx_buffer.Data, data, tx_buffer.DataLen);
    int send_state = VCI_Transmit(VCI_USBCAN2, device_id, m_can_id, &tx_buffer, 1);
    return send_state;
}

int serial_can_receive(BYTE* data, unsigned int *receive_id, DWORD device_id, DWORD m_can_id)
{
    VCI_CAN_OBJ rx_buffer;
    int receive_state = VCI_Receive(VCI_USBCAN2, device_id, m_can_id, &rx_buffer, 1, 10);
    if (rx_buffer.DataLen == 8)
        memcpy(data, rx_buffer.Data, rx_buffer.DataLen);
    *receive_id = rx_buffer.ID;
    return receive_state;
}

int serial_can_close(DWORD device_id)
{
    VCI_CloseDevice(VCI_USBCAN2, device_id);
}