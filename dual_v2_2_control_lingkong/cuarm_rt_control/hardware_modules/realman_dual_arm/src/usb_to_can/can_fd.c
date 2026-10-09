#include "can_fd.h"
#include "zcan.h"

#define CUARM_MAX_USBCANFD_DEVICES 2
#define CUARM_USBCANFD_SN_LENGTH 20
#define CUARM_DEFAULT_USBCANFD_SN_0 "6472C8BAD0F20BF466B0"
#define CUARM_DEFAULT_USBCANFD_SN_1 "B961A51020E9075418A0"

static int g_device_map[CUARM_MAX_USBCANFD_DEVICES] = {-1, -1};
static int g_physical_device_claimed[CUARM_MAX_USBCANFD_DEVICES] = {0, 0};

static int physical_device_index(int logical_dev_idx) {
    if (logical_dev_idx < 0 ||
        logical_dev_idx >= CUARM_MAX_USBCANFD_DEVICES) {
        return -1;
    }
    return g_device_map[logical_dev_idx];
}

static const char* expected_serial_for_logical_device(int logical_dev_idx) {
    const char* configured_serial = NULL;
    const char* default_serial = NULL;
    if (logical_dev_idx == 0) {
        configured_serial = getenv("CUARM_USBCANFD_SN_0");
        default_serial = CUARM_DEFAULT_USBCANFD_SN_0;
    } else if (logical_dev_idx == 1) {
        configured_serial = getenv("CUARM_USBCANFD_SN_1");
        default_serial = CUARM_DEFAULT_USBCANFD_SN_1;
    }
    if (configured_serial && configured_serial[0] != '\0') {
        return configured_serial;
    }
    if (default_serial) return default_serial;
    return NULL;
}

static int resolve_device_by_serial(
    int logical_dev_idx,
    const char* expected_serial,
    ZCAN_DEV_INF* matched_info
) {
    if (!expected_serial || expected_serial[0] == '\0') {
        return logical_dev_idx;
    }
    if (strlen(expected_serial) != CUARM_USBCANFD_SN_LENGTH) {
        fprintf(
            stderr,
            "[CANMAP] Logical device %d expected serial must contain exactly %d characters.\n",
            logical_dev_idx,
            CUARM_USBCANFD_SN_LENGTH
        );
        return -1;
    }

    for (int candidate = 0;
         candidate < CUARM_MAX_USBCANFD_DEVICES;
         ++candidate) {
        if (g_physical_device_claimed[candidate]) continue;
        if (!VCI_OpenDevice(USBCANFD, candidate, 0)) continue;

        ZCAN_DEV_INF info;
        memset(&info, 0, sizeof(info));
        const int info_ok = VCI_ReadBoardInfo(USBCANFD, candidate, &info);
        VCI_CloseDevice(USBCANFD, candidate);
        if (!info_ok) continue;
        if (memcmp(
                info.sn,
                expected_serial,
                CUARM_USBCANFD_SN_LENGTH
            ) == 0) {
            if (matched_info) *matched_info = info;
            return candidate;
        }
    }

    fprintf(
        stderr,
        "[CANMAP] Expected serial %s for logical device %d was not found; refusing to open CAN.\n",
        expected_serial,
        logical_dev_idx
    );
    return -1;
}

// CANFD初始化（波特率固定为仲裁段1Mbps，数据段5Mbps）
int canfd_init(int dev_idx, int chn_idx) {
    if (dev_idx < 0 || dev_idx >= CUARM_MAX_USBCANFD_DEVICES) {
        fprintf(stderr, "[CANMAP] Invalid logical device index %d.\n", dev_idx);
        return -1;
    }

    const char* expected_serial =
        expected_serial_for_logical_device(dev_idx);
    ZCAN_DEV_INF device_info;
    memset(&device_info, 0, sizeof(device_info));
    const int physical_dev_idx = resolve_device_by_serial(
        dev_idx,
        expected_serial,
        &device_info
    );
    if (physical_dev_idx < 0) return -1;

    // 打开设备
    if (!VCI_OpenDevice(USBCANFD, physical_dev_idx, 0)) {
        fprintf(stderr, "Failed to open device\n");
        return -1;
    }
    if (!VCI_ReadBoardInfo(USBCANFD, physical_dev_idx, &device_info)) {
        fprintf(stderr, "[CANMAP] Failed to read device serial number.\n");
        VCI_CloseDevice(USBCANFD, physical_dev_idx);
        return -1;
    }
    if (expected_serial &&
        memcmp(
            device_info.sn,
            expected_serial,
            CUARM_USBCANFD_SN_LENGTH
        ) != 0) {
        fprintf(stderr, "[CANMAP] Device serial changed while opening; refusing CAN startup.\n");
        VCI_CloseDevice(USBCANFD, physical_dev_idx);
        return -1;
    }

    // 固定波特率配置（仲裁段1Mbps，数据段5Mbps）
    ZCAN_INIT init_cfg = {
        .clk = 60000000,    // 设备时钟60MHz
        .mode = 0,          // 正常模式
        
        // 仲裁段配置（1Mbps）
        .aset = {
            .tseg1 = 2,    // 时间段1
            .tseg2 = 0,     // 时间段2
            .sjw = 0,       // 同步跳转宽度
            .smp = 0,       // 采样次数
            .brp = 11        // 波特率分频
        },
        
        // 数据段配置（5Mbps）
        .dset = {
            .tseg1 = 1,     // 时间段1
            .tseg2 = 0,     // 时间段2
            .sjw = 0,       // 同步跳转宽度
            .smp = 0,       // 采样次数
            .brp = 2        // 波特率分频
        }
    };

    // 初始化通道
    if (!VCI_InitCAN(USBCANFD, physical_dev_idx, chn_idx, &init_cfg)) {
        fprintf(stderr, "Failed to initialize channel\n");
        VCI_CloseDevice(USBCANFD, physical_dev_idx);
        return -1;
    }

    // 启动通道
    if (!VCI_StartCAN(USBCANFD, physical_dev_idx, chn_idx)) {
        fprintf(stderr, "Failed to start channel\n");
        VCI_CloseDevice(USBCANFD, physical_dev_idx);
        return -1;
    }

    // 默认启用终端电阻
    uint32_t term = 1;
    VCI_SetReference(
        USBCANFD,
        physical_dev_idx,
        chn_idx,
        CMD_CAN_TRES,
        &term
    );

    g_device_map[dev_idx] = physical_dev_idx;
    g_physical_device_claimed[physical_dev_idx] = 1;
    fprintf(
        stdout,
        "[CANMAP] logical_device=%d physical_device=%d SN=%.*s\n",
        dev_idx,
        physical_dev_idx,
        CUARM_USBCANFD_SN_LENGTH,
        (const char*)device_info.sn
    );
    fflush(stdout);

    return 0;
}

// CANFD发送单帧（参数不变）
int canfd_send(int dev_idx, int chn_idx, uint32_t id, 
               const uint8_t* data, uint8_t len) {
    const int physical_dev_idx = physical_device_index(dev_idx);
    if (physical_dev_idx < 0) return -1;
    if (len > CANFD_MAX_DATA_LEN) {
        fprintf(stderr, "Data length exceeds 64 bytes\n");
        return -1;
    }
    ZCAN_FD_MSG frame = {
        .hdr = {
            .inf = {
                .fmt = 1,       // CANFD格式
                .sef = 0,  // 扩展帧标识
                .brs = 1        // 启用加速传输
            },
            .id = id,
            .chn = (uint8_t)chn_idx,
            .len = len
        }
    };
    memcpy(frame.dat, data, len);
    // printf("send: can_id 0x%x, dlc 0x%x data: ",frame.hdr.id, frame.hdr.len);
    // for (int i = 0;i<frame.hdr.len;++i)
    // {
    //     printf("0x%x ",frame.dat[i]);
    // }
    // printf("\n");
    int send_status = VCI_TransmitFD(
        USBCANFD,
        physical_dev_idx,
        chn_idx,
        &frame,
        1
    );
    // printf("send status : %d\n", send_status);
    
    return send_status;
}

// CANFD接收单帧（参数不变）
int canfd_receive(int dev_idx, int chn_idx, uint32_t* id, 
                  uint8_t* data, uint8_t* len, 
                  int timeout_ms) {
    const int physical_dev_idx = physical_device_index(dev_idx);
    if (physical_dev_idx < 0) return -1;
    ZCAN_FD_MSG frame;
    int received = VCI_ReceiveFD(
        USBCANFD,
        physical_dev_idx,
        chn_idx,
        &frame,
        1,
        timeout_ms
    );
    
    if (received < 0) return -1;
    //printf("frame.hdr.id %x, frame.hdr.len %x\n", frame.hdr.id, frame.hdr.len);
    *id = frame.hdr.id;
    *len = frame.hdr.len;

    if (data) 
    {  
        memcpy(data, frame.dat, frame.hdr.len);
        // printf("receive: can_id 0x%x, dlc 0x%x data: ",frame.hdr.id, frame.hdr.len);
        // for (int i = 0;i<frame.hdr.len;++i)
        // {
        //     printf("0x%x ",data[i]);
        // }
        // printf("\n------------\n");
    }

    return 0;
}

// CANFD关闭设备（参数不变）
void canfd_close(int dev_idx, int chn_idx) {
    const int physical_dev_idx = physical_device_index(dev_idx);
    if (physical_dev_idx < 0) return;
    VCI_ResetCAN(USBCANFD, physical_dev_idx, chn_idx);
    VCI_CloseDevice(USBCANFD, physical_dev_idx);
    if (dev_idx >= 0 && dev_idx < CUARM_MAX_USBCANFD_DEVICES) {
        g_device_map[dev_idx] = -1;
    }
    if (physical_dev_idx < CUARM_MAX_USBCANFD_DEVICES) {
        g_physical_device_claimed[physical_dev_idx] = 0;
    }
}