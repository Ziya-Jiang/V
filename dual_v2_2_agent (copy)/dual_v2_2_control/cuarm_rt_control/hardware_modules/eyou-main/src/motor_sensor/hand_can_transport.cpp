#include "hand_can_transport.h"

#include "eu_canable.h"

#include <array>
#include <chrono>
#include <iostream>
#include <vector>

#ifdef ARM_ARCH
    constexpr int kPreferredCanableDeviceIndex = 2;
#else
    constexpr int kPreferredCanableDeviceIndex = 0;
#endif

namespace {
std::vector<int> canableDeviceCandidates() {
    std::vector<int> candidates;
    const std::array<int, 5> preferred_order = {kPreferredCanableDeviceIndex, 0, 1, 2, 3};
    for (int index : preferred_order) {
        bool exists = false;
        for (int candidate : candidates) {
            if (candidate == index) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            candidates.push_back(index);
        }
    }
    return candidates;
}

void printCanableCandidates(const std::vector<int>& candidates) {
    std::cerr << "[";
    for (size_t i = 0; i < candidates.size(); ++i) {
        std::cerr << candidates[i] << (i + 1 == candidates.size() ? "" : ",");
    }
    std::cerr << "]";
}

HandCanFrame toHandFrame(const canable_CanMsg& msg) {
    HandCanFrame frame;
    frame.id = msg.id;
    frame.is_extended = msg.isExtended;
    frame.dlc = msg.dlc;
    for (uint8_t i = 0; i < frame.dlc && i < frame.data.size(); ++i) {
        frame.data[i] = msg.data[i];
    }
    return frame;
}

canable_CanMsg toCanableFrame(const HandCanFrame& frame) {
    canable_CanMsg msg = {};
    msg.id = frame.id;
    msg.isExtended = frame.is_extended;
    msg.dlc = frame.dlc;
    for (uint8_t i = 0; i < frame.dlc && i < frame.data.size(); ++i) {
        msg.data[i] = frame.data[i];
    }
    return msg;
}
}

CanableHandTransport::CanableHandTransport(const int* hand_ids) {
    (void)hand_ids;

    const std::vector<int> candidates = canableDeviceCandidates();
    for (int index : candidates) {
        const int ret = canable_initDLL(index, canable_Baudrate_1000);
        std::cerr << "try CANABLE device_index=" << index << " ret=" << ret << std::endl;
        if (ret == CANABLE_SUCCESS) {
            device_index_ = index;
            std::cout << "CANABLE init success, device_index=" << device_index_ << std::endl;
            return;
        }
    }

    std::cerr << "CANABLE init failed for all device indices ";
    printCanableCandidates(candidates);
    std::cerr << std::endl;
}

CanableHandTransport::~CanableHandTransport() {
    if (device_index_ >= 0) {
        canable_freeDLL(device_index_);
    }
}

int CanableHandTransport::request(int hand_idx,
                                  uint32_t expected_hand_id,
                                  const HandCanFrame& request_frame,
                                  HandCanFrame& response,
                                  int timeout_ms) {
    if (hand_idx < 0 || hand_idx > 1) {
        return HAND_CAN_FAILED_RECEIVE;
    }
    if (device_index_ < 0) {
        return HAND_CAN_FAILED_RECEIVE;
    }

    canable_CanMsg msg = toCanableFrame(request_frame);
    int ret = canable_writeMessage(device_index_, &msg);
    if (ret != CANABLE_SUCCESS) {
        return ret;
    }

    return waitForResponse(hand_idx, expected_hand_id, response, timeout_ms);
}

int CanableHandTransport::waitForResponse(int hand_idx,
                                          uint32_t expected_hand_id,
                                          HandCanFrame& response,
                                          int timeout_ms) {
    if (hand_idx < 0 || hand_idx > 1) {
        return HAND_CAN_FAILED_RECEIVE;
    }
    if (device_index_ < 0) {
        return HAND_CAN_FAILED_RECEIVE;
    }

    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed_ms >= timeout_ms) {
            return HAND_CAN_FAILED_RECEIVE;
        }

        canable_CanMsg msg = {};
        const int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
        int ret = canable_readMessage(device_index_, &msg, remaining_ms);
        if (ret != CANABLE_SUCCESS) {
            continue;
        }

        HandCanFrame received = toHandFrame(msg);
        if ((received.id & 0x3FFF) == (expected_hand_id & 0x3FFF)) {
            response = received;
            return HAND_CAN_SUCCESS;
        }
    }
}
