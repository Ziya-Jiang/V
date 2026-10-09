#include "serial_can.h"
// int serial_can_open(DWORD device_id, DWORD channel)
// {
//     DWORD m_device_type = VCI_USBCAN2;
//     DWORD m_device_id = device_id;
//     DWORD m_can_id = channel;
//     //printf("%d %d %d\n", m_device_type, m_device_id, m_can_id);
//     DWORD dwRel = VCI_OpenDevice(m_device_type, m_device_id, 0);
//     if (dwRel != 1) {
//         printf("Open Device Failed\n");
//         return -1;
//     }

//     VCI_BOARD_INFO vbi;
//     dwRel = VCI_ReadBoardInfo(m_device_type, m_device_id, &vbi);

//     if (dwRel != 1) {
//         printf("Get Board Info Failed\n");
//         return -2;
//     } else {
//         printf("Get VCI_ReadBoardInfo success!\n");
//         printf("hw_Type:%c", vbi.str_hw_Type[0]);
//         printf("%c", vbi.str_hw_Type[1]);
//         printf("%c", vbi.str_hw_Type[2]);
//         printf("%c", vbi.str_hw_Type[3]);
//         printf("%c", vbi.str_hw_Type[4]);
//         printf("%c", vbi.str_hw_Type[5]);
//         printf("%c", vbi.str_hw_Type[6]);
//         printf("%c", vbi.str_hw_Type[7]);
//         printf("%c", vbi.str_hw_Type[8]);
//         printf("%c", vbi.str_hw_Type[9]);
//         printf("\n");

//         printf("Firmware Version:V");
//         printf("%x", (vbi.fw_Version&0xF00)>>8);
//         printf(".");
//         printf("%x", (vbi.fw_Version&0xF0)>>4);
//         printf("%x", vbi.fw_Version&0xF);
//         printf("\n");	
//     }

//     VCI_INIT_CONFIG vic;
//     vic.AccCode = 0x80000008;
//     vic.AccMask = 0xFFFFFFFF;
//     vic.Filter = 1;
//     vic.Timing0 = 0x00;  //0014 = 1000Kbps
//     vic.Timing1 = 0x14;
//     // vic.Timing0 = 0x04;  
//     // vic.Timing1 = 0x1C; //041C = 100kbps
//     //vic.Timing1 = 0x1C;  //001C = 500Kbps
//     vic.Mode = 0; // 0 =normal; 1=listenonly; 2=selftesting

//     dwRel = VCI_InitCAN(m_device_type, m_device_id, m_can_id, &vic);
//     if (dwRel != 1) {
//         VCI_CloseDevice(m_device_type, m_device_id);
//         printf("Failed to Init CAN \n");
//         return -3;
//     } else {
//         printf("Init CAN %d .. OK\n", m_can_id);
//     }

//     if (VCI_StartCAN(m_device_type, m_device_id, m_can_id) != 1) {
//         VCI_CloseDevice(m_device_type, m_device_id);
//         printf("Failed to Start CAN \n");
//         return -4;
//     } else {
//         printf("Start CAN %d .. OK\n", m_can_id);
//     }
//     return 0;
// }
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
        printf("%d \n", (channels & (1 << ch)));
        if (!(channels & (1 << ch))) continue;
        printf("ch %d \n", ch);
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
    //DEBUG_PRINT("send: can_id 0x%x, dlc 0x%x data: \n",tx_buffer.ID, tx_buffer.DataLen);
    for (int i = 0;i<tx_buffer.DataLen;++i)
    {
        //DEBUG_PRINT("0x%x ",tx_buffer.Data[i]);
    }
    //DEBUG_PRINT("\n\n");
    
    int send_state = VCI_Transmit(VCI_USBCAN2, device_id, m_can_id, &tx_buffer, 1);
    return send_state;
}

int serial_can_receive(BYTE* data, unsigned int *receive_id, DWORD device_id, DWORD m_can_id)
{
    VCI_CAN_OBJ rx_buffer;
    int receive_state = VCI_Receive(VCI_USBCAN2, device_id, m_can_id, &rx_buffer, 1, 10);
    if (rx_buffer.DataLen == 8)
        memcpy(data, rx_buffer.Data, rx_buffer.DataLen);
    //DEBUG_PRINT("receive: can_id 0x%x, dlc 0x%x data: \n",rx_buffer.ID, rx_buffer.DataLen);
    *receive_id = rx_buffer.ID;
    for (int i = 0;i<rx_buffer.DataLen;++i)
    {
        //DEBUG_PRINT("0x%x ",data[i]);
    }
    //DEBUG_PRINT("\n\n");
    return receive_state;
}

int serial_can_close(DWORD device_id)
{
    VCI_CloseDevice(VCI_USBCAN2, device_id);
}