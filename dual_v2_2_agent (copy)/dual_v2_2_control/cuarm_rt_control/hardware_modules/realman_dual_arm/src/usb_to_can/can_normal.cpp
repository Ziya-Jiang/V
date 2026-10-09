#include "can_normal.h"
#include "controlcan.h"
int can_normal_init(unsigned int channels, USB_CAN_DEVICE device_type)
{
    switch (device_type)
    {
    case EU_CANABLE:
    {
        if (CANABLE_SUCCESS != canable_initDLL(channels, canable_Baudrate_1000))
        {
            printf("Init CANABLE Device %d Failed\n", channels);
            return -1;
        }
        else
        {
            printf("CANABLE Device %d init success\n", channels);
        }
        break;
    }
    case CAN_ANLYSTII:
    {
        const unsigned int DEVICE_TYPE = VCI_USBCAN2;
        const unsigned int DEVICE_ID = 0;  // 固定设备号为0

        // 打开设备
        if (VCI_OpenDevice(DEVICE_TYPE, DEVICE_ID, 0) != 1) {
            printf("Open CAN NORMAL Device Failed\n");
            return -1;
        }

        // 初始化配置参数
        VCI_INIT_CONFIG vic = {
            .AccCode = 0x80000008,
            .AccMask = 0xFFFFFFFF,
            .Filter = 1,
            .Timing0 = 0x00,  // 1000Kbps
            .Timing1 = 0x14,
            .Mode = 0
        };

        // 直接处理两个通道
        for (unsigned int ch = 0; ch < 2; ch++) {
            if (!(channels & (1 << ch))) continue;
            if (VCI_InitCAN(DEVICE_TYPE, DEVICE_ID, ch, &vic) != 1) {
                VCI_CloseDevice(DEVICE_TYPE, DEVICE_ID);
                printf("Init CAN%d failed\n", ch);
                return -2;
            }
            if (VCI_StartCAN(DEVICE_TYPE, DEVICE_ID, ch) != 1) {
                VCI_CloseDevice(DEVICE_TYPE, DEVICE_ID);
                printf("Start CAN%d failed\n", ch);
                return -3;
            }
            printf("CAN%d init & start success\n", ch);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

int can_normal_send(USB_CAN_DEVICE device_type, unsigned int can_id, const unsigned char* data, unsigned int device_id, unsigned int m_can_id)
{
    switch (device_type)
    {
    case EU_CANABLE:
    {
        canable_CanMsg write_msg;
        write_msg.isExtended = 0;
        write_msg.isRTR = 0;
        write_msg.dlc = 8;
        write_msg.id = can_id;
        write_msg.data[0] = 0x55;
        memcpy(write_msg.data, data, write_msg.dlc);
        for (int i = 1; i < 8; ++i) {
            write_msg.data[i] = 0x00;
        }
        int ret = canable_writeMessage(device_id, &write_msg);
        if (ret != CANABLE_SUCCESS) {
            printf("Send CANABLE Message Failed,Error Code: %d\n", ret);
            return -1;
        }
        return 0;
    }
    case CAN_ANLYSTII:
    {
        VCI_CAN_OBJ tx_buffer;
        tx_buffer.ID = can_id;
        tx_buffer.SendType = 0;
        tx_buffer.RemoteFlag = 0;
        tx_buffer.ExternFlag = 0;
        tx_buffer.DataLen = 8;

        memcpy(tx_buffer.Data, data, tx_buffer.DataLen);
        int send_state = VCI_Transmit(VCI_USBCAN2, 0, m_can_id, &tx_buffer, 1);
        return send_state;
    }
    default:
        return -1; // 添加默认返回值
    }
}
int can_normal_receive(USB_CAN_DEVICE device_type, unsigned char* data, unsigned int *receive_id, unsigned int device_id, unsigned int m_can_id)
{
    switch (device_type)
    {
    case EU_CANABLE:
    {
        canable_CanMsg read_msg;
        int ret = canable_readMessage(device_id, &read_msg, 10);
        if (ret != CANABLE_SUCCESS) {
            //printf("Device_id %d Read CANABLE Message Failed,Error Code: %d\n",device_id, ret);
            return -1;
        }
        if (read_msg.dlc == 8)
            memcpy(data, read_msg.data, read_msg.dlc);
        *receive_id = read_msg.id;
        return 0;
    }
    case CAN_ANLYSTII:
    {
        VCI_CAN_OBJ rx_buffer;
        int receive_state = VCI_Receive(VCI_USBCAN2, 0, m_can_id, &rx_buffer, 1, 10);
        if (rx_buffer.DataLen == 8)
            memcpy(data, rx_buffer.Data, rx_buffer.DataLen);
        *receive_id = rx_buffer.ID;
        return receive_state;
    }
    default:
        return -1; // 添加默认返回值
    }
}

int can_normal_close(USB_CAN_DEVICE device_type, unsigned int device_id)
{
    switch (device_type)
    {
    case EU_CANABLE:
    {
        if (CANABLE_SUCCESS != canable_freeDLL(device_id))
        {
            printf("Close CANABLE Device %d Failed\n", device_id);
            return -1;
        }
        else
        {
            printf("CANABLE Device %d close success\n", device_id);
        }
        return 0;
    }
    case CAN_ANLYSTII:
    {
        VCI_CloseDevice(VCI_USBCAN2, device_id);
        return 0; // 假设关闭成功
    }
    default:
        return -1; // 添加默认返回值
    }
}

