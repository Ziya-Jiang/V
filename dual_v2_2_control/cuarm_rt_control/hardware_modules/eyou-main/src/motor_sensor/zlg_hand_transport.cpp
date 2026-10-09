#include "zlg_hand_transport.h"

#include "zcan.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <dlfcn.h>

#ifndef USBCANFD
#define USBCANFD 33
#endif

namespace {
constexpr int kChannelCount = 2;

using VciOpenDeviceFn = U32 (*)(U32, U32, U32);
using VciCloseDeviceFn = U32 (*)(U32, U32);
using VciInitCanFn = U32 (*)(U32, U32, U32, ZCAN_INIT*);
using VciStartCanFn = U32 (*)(U32, U32, U32);
using VciResetCanFn = U32 (*)(U32, U32, U32);
using VciSetReferenceFn = U32 (*)(U32, U32, U32, U32, void*);
using VciTransmitFdFn = U32 (*)(U32, U32, U32, ZCAN_FD_MSG*, U32);
using VciReceiveFdFn = U32 (*)(U32, U32, U32, ZCAN_FD_MSG*, U32, U32);
using VciTransmitCan20Fn = U32 (*)(U32, U32, U32, ZCAN_20_MSG*, U32);
using VciReceiveCan20Fn = U32 (*)(U32, U32, U32, ZCAN_20_MSG*, U32, U32);

struct ZlgApi {
    void* handle = nullptr;
    VciOpenDeviceFn openDevice = nullptr;
    VciCloseDeviceFn closeDevice = nullptr;
    VciInitCanFn initCan = nullptr;
    VciStartCanFn startCan = nullptr;
    VciResetCanFn resetCan = nullptr;
    VciSetReferenceFn setReference = nullptr;
    VciTransmitFdFn transmitFd = nullptr;
    VciReceiveFdFn receiveFd = nullptr;
    VciTransmitCan20Fn transmitCan20 = nullptr;
    VciReceiveCan20Fn receiveCan20 = nullptr;
    bool loaded = false;
};

template <typename Fn>
Fn loadZlgSymbol(void* handle, const char* name) {
    void* symbol = dlsym(handle, name);
    if (!symbol) {
        std::cerr << "ZLG dlsym failed: " << name << ": " << dlerror() << std::endl;
        return nullptr;
    }
    return reinterpret_cast<Fn>(symbol);
}

ZlgApi& zlgApi() {
    static ZlgApi api;
    if (api.loaded) {
        return api;
    }

    const char* candidates[] = {
        "/home/lite_arm/xyh/eyou_dual_arm-main/lib/canfd/x86/libusbcanfd.so",
        "/usr/local/lib/libusbcanfd.so.1.0.10",
        "libusbcanfd.so.1.0.10",
        "libusbcanfd.so",
    };
    for (const char* path : candidates) {
        api.handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
        if (api.handle) {
            break;
        }
    }
    if (!api.handle) {
        std::cerr << "ZLG dlopen libusbcanfd failed: " << dlerror() << std::endl;
        api.loaded = true;
        return api;
    }

    api.openDevice = loadZlgSymbol<VciOpenDeviceFn>(api.handle, "VCI_OpenDevice");
    api.closeDevice = loadZlgSymbol<VciCloseDeviceFn>(api.handle, "VCI_CloseDevice");
    api.initCan = loadZlgSymbol<VciInitCanFn>(api.handle, "VCI_InitCAN");
    api.startCan = loadZlgSymbol<VciStartCanFn>(api.handle, "VCI_StartCAN");
    api.resetCan = loadZlgSymbol<VciResetCanFn>(api.handle, "VCI_ResetCAN");
    api.setReference = loadZlgSymbol<VciSetReferenceFn>(api.handle, "VCI_SetReference");
    api.transmitFd = loadZlgSymbol<VciTransmitFdFn>(api.handle, "VCI_TransmitFD");
    api.receiveFd = loadZlgSymbol<VciReceiveFdFn>(api.handle, "VCI_ReceiveFD");
    api.transmitCan20 = loadZlgSymbol<VciTransmitCan20Fn>(api.handle, "VCI_Transmit");
    api.receiveCan20 = loadZlgSymbol<VciReceiveCan20Fn>(api.handle, "VCI_Receive");
    api.loaded = true;
    return api;
}

bool zlgApiReady() {
    ZlgApi& api = zlgApi();
    return api.openDevice && api.closeDevice && api.initCan && api.startCan &&
           api.resetCan && api.setReference && api.transmitFd && api.receiveFd &&
           api.transmitCan20 && api.receiveCan20;
}

ZCAN_INIT makeDefaultCanFdInit() {
    ZCAN_INIT init_cfg = {};
    init_cfg.clk = 60000000;
    init_cfg.mode = 0;

    // Arbitration phase: 1 Mbps, matching the ZLG CAN-FD sample configuration.
    init_cfg.aset.tseg1 = 2;
    init_cfg.aset.tseg2 = 0;
    init_cfg.aset.sjw = 0;
    init_cfg.aset.smp = 0;
    init_cfg.aset.brp = 11;

    // Data phase: 5 Mbps for CAN FD.
    init_cfg.dset.tseg1 = 1;
    init_cfg.dset.tseg2 = 0;
    init_cfg.dset.sjw = 0;
    init_cfg.dset.smp = 0;
    init_cfg.dset.brp = 2;
    return init_cfg;
}

int initZlgChannel(int dev_idx, int chn_idx) {
    if (!zlgApiReady()) {
        return -1;
    }

    if (!zlgApi().openDevice(USBCANFD, dev_idx, 0)) {
        std::cerr << "ZLG OpenDevice failed: dev=" << dev_idx << std::endl;
        return -1;
    }

    ZCAN_INIT init_cfg = makeDefaultCanFdInit();
    if (!zlgApi().initCan(USBCANFD, dev_idx, chn_idx, &init_cfg)) {
        std::cerr << "ZLG InitCAN failed: dev=" << dev_idx << " chn=" << chn_idx << std::endl;
        zlgApi().closeDevice(USBCANFD, dev_idx);
        return -1;
    }

    if (!zlgApi().startCan(USBCANFD, dev_idx, chn_idx)) {
        std::cerr << "ZLG StartCAN failed: dev=" << dev_idx << " chn=" << chn_idx << std::endl;
        zlgApi().closeDevice(USBCANFD, dev_idx);
        return -1;
    }

    uint32_t term = 1;
    if (!zlgApi().setReference(USBCANFD, dev_idx, chn_idx, CMD_CAN_TRES, &term)) {
        std::cerr << "ZLG Set terminal resistor failed: dev=" << dev_idx << " chn=" << chn_idx << std::endl;
    }
    return HAND_CAN_SUCCESS;
}

void resetZlgChannel(int dev_idx, int chn_idx) {
    zlgApi().resetCan(USBCANFD, dev_idx, chn_idx);
}

void closeZlgDevice(int dev_idx) {
    zlgApi().closeDevice(USBCANFD, dev_idx);
}


template <typename Fn>
void forEachUniqueDevice(const std::array<int, kChannelCount>& dev_indices, Fn fn) {
    for (int i = 0; i < kChannelCount; ++i) {
        bool seen = false;
        for (int j = 0; j < i; ++j) {
            if (dev_indices[i] == dev_indices[j]) {
                seen = true;
                break;
            }
        }
        if (!seen) {
            fn(dev_indices[i]);
        }
    }
}

template <typename Fn>
void forEachUniqueChannel(const std::array<int, kChannelCount>& dev_indices,
                          const std::array<int, kChannelCount>& chn_indices,
                          Fn fn) {
    for (int i = 0; i < kChannelCount; ++i) {
        bool seen = false;
        for (int j = 0; j < i; ++j) {
            if (dev_indices[i] == dev_indices[j] && chn_indices[i] == chn_indices[j]) {
                seen = true;
                break;
            }
        }
        if (!seen) {
            fn(dev_indices[i], chn_indices[i]);
        }
    }
}

ZCAN_FD_MSG toZlgFdFrame(const HandCanFrame& frame, int chn_idx) {
    ZCAN_FD_MSG msg = {};
    msg.hdr.inf.fmt = 1;
    msg.hdr.inf.sef = frame.is_extended ? 1 : 0;
    msg.hdr.inf.brs = 1;
    msg.hdr.id = frame.id;
    msg.hdr.chn = static_cast<U8>(chn_idx);
    msg.hdr.len = frame.dlc;
    std::memcpy(msg.dat, frame.data.data(), frame.dlc);
    return msg;
}

HandCanFrame fromZlgFdFrame(const ZCAN_FD_MSG& msg) {
    HandCanFrame frame;
    frame.id = msg.hdr.id;
    frame.is_extended = msg.hdr.inf.sef == 1;
    frame.dlc = std::min<uint8_t>(msg.hdr.len, frame.data.size());
    std::memcpy(frame.data.data(), msg.dat, frame.dlc);
    return frame;
}

ZCAN_20_MSG toZlgCan20Frame(const HandCanFrame& frame, int chn_idx) {
    ZCAN_20_MSG msg = {};
    msg.hdr.inf.fmt = 0;
    msg.hdr.inf.sef = frame.is_extended ? 1 : 0;
    msg.hdr.id = frame.id;
    msg.hdr.chn = static_cast<U8>(chn_idx);
    msg.hdr.len = frame.dlc;
    std::memcpy(msg.dat, frame.data.data(), frame.dlc);
    return msg;
}

HandCanFrame fromZlgCan20Frame(const ZCAN_20_MSG& msg) {
    HandCanFrame frame;
    frame.id = msg.hdr.id;
    frame.is_extended = msg.hdr.inf.sef == 1;
    frame.dlc = std::min<uint8_t>(msg.hdr.len, frame.data.size());
    std::memcpy(frame.data.data(), msg.dat, frame.dlc);
    return frame;
}

int remainingTimeoutMs(std::chrono::steady_clock::time_point start, int timeout_ms) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    return std::max(0, timeout_ms - static_cast<int>(elapsed));
}
}

ZlgCanFdHandTransport::ZlgCanFdHandTransport(int dev_idx,
                                             int chn_idx,
                                             bool initialize,
                                             bool close_on_destroy)
    : ZlgCanFdHandTransport({dev_idx, dev_idx}, {chn_idx, chn_idx}, initialize, close_on_destroy) {
}

ZlgCanFdHandTransport::ZlgCanFdHandTransport(const std::array<int, 2>& dev_indices,
                                             const std::array<int, 2>& chn_indices,
                                             bool initialize,
                                             bool close_on_destroy)
    : dev_indices_(dev_indices),
      chn_indices_(chn_indices),
      close_on_destroy_(close_on_destroy) {
    initialized_ = true;
    if (initialize) {
        forEachUniqueChannel(dev_indices_, chn_indices_, [this](int dev_idx, int chn_idx) {
            if (initZlgChannel(dev_idx, chn_idx) != HAND_CAN_SUCCESS) {
                initialized_ = false;
            }
        });
    }
}

ZlgCanFdHandTransport::~ZlgCanFdHandTransport() {
    if (close_on_destroy_ && initialized_) {
        forEachUniqueChannel(dev_indices_, chn_indices_, [](int dev_idx, int chn_idx) {
            resetZlgChannel(dev_idx, chn_idx);
        });
        forEachUniqueDevice(dev_indices_, [](int dev_idx) {
            closeZlgDevice(dev_idx);
        });
    }
}

int ZlgCanFdHandTransport::request(int hand_idx,
                                   uint32_t expected_hand_id,
                                   const HandCanFrame& request_frame,
                                   HandCanFrame& response,
                                   int timeout_ms) {
    if (!initialized_ || hand_idx < 0 || hand_idx >= kChannelCount || request_frame.dlc > request_frame.data.size()) {
        return HAND_CAN_FAILED_RECEIVE;
    }

    ZCAN_FD_MSG msg = toZlgFdFrame(request_frame, chn_indices_[hand_idx]);
    const U32 sent = zlgApi().transmitFd(USBCANFD, dev_indices_[hand_idx], chn_indices_[hand_idx], &msg, 1);
    if (sent != 1) {
        std::cerr << "ZLG TransmitFD failed: dev=" << dev_indices_[hand_idx]
                  << " chn=" << chn_indices_[hand_idx] << std::endl;
        return -1;
    }
    return waitForResponse(hand_idx, expected_hand_id, response, timeout_ms);
}

int ZlgCanFdHandTransport::waitForResponse(int hand_idx,
                                           uint32_t expected_hand_id,
                                           HandCanFrame& response,
                                           int timeout_ms) {
    if (!initialized_ || hand_idx < 0 || hand_idx >= kChannelCount) {
        return HAND_CAN_FAILED_RECEIVE;
    }

    auto start = std::chrono::steady_clock::now();
    while (remainingTimeoutMs(start, timeout_ms) > 0) {
        ZCAN_FD_MSG msg = {};
        const U32 received = zlgApi().receiveFd(USBCANFD,
                                           dev_indices_[hand_idx],
                                           chn_indices_[hand_idx],
                                           &msg,
                                           1,
                                           remainingTimeoutMs(start, timeout_ms));
        if (received == 0) {
            continue;
        }

        HandCanFrame frame = fromZlgFdFrame(msg);
        if (frame.is_extended && frame.dlc <= frame.data.size() &&
            (frame.id & 0x3FFF) == (expected_hand_id & 0x3FFF)) {
            response = frame;
            return HAND_CAN_SUCCESS;
        }
    }
    return HAND_CAN_FAILED_RECEIVE;
}

ZlgCan20HandTransport::ZlgCan20HandTransport(int dev_idx,
                                             int chn_idx,
                                             bool initialize,
                                             bool close_on_destroy)
    : ZlgCan20HandTransport({dev_idx, dev_idx}, {chn_idx, chn_idx}, initialize, close_on_destroy) {
}

ZlgCan20HandTransport::ZlgCan20HandTransport(const std::array<int, 2>& dev_indices,
                                             const std::array<int, 2>& chn_indices,
                                             bool initialize,
                                             bool close_on_destroy)
    : dev_indices_(dev_indices),
      chn_indices_(chn_indices),
      close_on_destroy_(close_on_destroy) {
    initialized_ = true;
    if (initialize) {
        forEachUniqueChannel(dev_indices_, chn_indices_, [this](int dev_idx, int chn_idx) {
            if (initZlgChannel(dev_idx, chn_idx) != HAND_CAN_SUCCESS) {
                initialized_ = false;
            }
        });
    }
}

ZlgCan20HandTransport::~ZlgCan20HandTransport() {
    if (close_on_destroy_ && initialized_) {
        forEachUniqueChannel(dev_indices_, chn_indices_, [](int dev_idx, int chn_idx) {
            resetZlgChannel(dev_idx, chn_idx);
        });
        forEachUniqueDevice(dev_indices_, [](int dev_idx) {
            closeZlgDevice(dev_idx);
        });
    }
}

int ZlgCan20HandTransport::request(int hand_idx,
                                   uint32_t expected_hand_id,
                                   const HandCanFrame& request_frame,
                                   HandCanFrame& response,
                                   int timeout_ms) {
    if (!initialized_ || hand_idx < 0 || hand_idx >= kChannelCount || request_frame.dlc > 8) {
        return HAND_CAN_FAILED_RECEIVE;
    }

    ZCAN_20_MSG msg = toZlgCan20Frame(request_frame, chn_indices_[hand_idx]);
    const U32 sent = zlgApi().transmitCan20(USBCANFD, dev_indices_[hand_idx], chn_indices_[hand_idx], &msg, 1);
    if (sent != 1) {
        std::cerr << "ZLG Transmit CAN2.0 failed: dev=" << dev_indices_[hand_idx]
                  << " chn=" << chn_indices_[hand_idx] << std::endl;
        return -1;
    }
    return waitForResponse(hand_idx, expected_hand_id, response, timeout_ms);
}

int ZlgCan20HandTransport::waitForResponse(int hand_idx,
                                           uint32_t expected_hand_id,
                                           HandCanFrame& response,
                                           int timeout_ms) {
    if (!initialized_ || hand_idx < 0 || hand_idx >= kChannelCount) {
        return HAND_CAN_FAILED_RECEIVE;
    }

    auto start = std::chrono::steady_clock::now();
    while (remainingTimeoutMs(start, timeout_ms) > 0) {
        ZCAN_20_MSG msg = {};
        const U32 received = zlgApi().receiveCan20(USBCANFD,
                                         dev_indices_[hand_idx],
                                         chn_indices_[hand_idx],
                                         &msg,
                                         1,
                                         remainingTimeoutMs(start, timeout_ms));
        if (received == 0) {
            continue;
        }

        HandCanFrame frame = fromZlgCan20Frame(msg);
        if (frame.is_extended && frame.dlc <= frame.data.size() &&
            (frame.id & 0x3FFF) == (expected_hand_id & 0x3FFF)) {
            response = frame;
            return HAND_CAN_SUCCESS;
        }
    }
    return HAND_CAN_FAILED_RECEIVE;
}
