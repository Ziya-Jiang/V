#ifndef ZLG_HAND_TRANSPORT_H
#define ZLG_HAND_TRANSPORT_H

#include "hand_can_transport.h"
#include <array>

class ZlgCanFdHandTransport : public IHandCanTransport {
public:
    ZlgCanFdHandTransport(int dev_idx,
                          int chn_idx,
                          bool initialize = true,
                          bool close_on_destroy = true);
    ZlgCanFdHandTransport(const std::array<int, 2>& dev_indices,
                          const std::array<int, 2>& chn_indices,
                          bool initialize = true,
                          bool close_on_destroy = true);
    ~ZlgCanFdHandTransport() override;

    int request(int hand_idx,
                uint32_t expected_hand_id,
                const HandCanFrame& request,
                HandCanFrame& response,
                int timeout_ms) override;

    int waitForResponse(int hand_idx,
                        uint32_t expected_hand_id,
                        HandCanFrame& response,
                        int timeout_ms) override;

private:
    std::array<int, 2> dev_indices_;
    std::array<int, 2> chn_indices_;
    bool close_on_destroy_;
    bool initialized_ = false;
};

class ZlgCan20HandTransport : public IHandCanTransport {
public:
    ZlgCan20HandTransport(int dev_idx,
                          int chn_idx,
                          bool initialize = true,
                          bool close_on_destroy = true);
    ZlgCan20HandTransport(const std::array<int, 2>& dev_indices,
                          const std::array<int, 2>& chn_indices,
                          bool initialize = true,
                          bool close_on_destroy = true);
    ~ZlgCan20HandTransport() override;

    int request(int hand_idx,
                uint32_t expected_hand_id,
                const HandCanFrame& request,
                HandCanFrame& response,
                int timeout_ms) override;

    int waitForResponse(int hand_idx,
                        uint32_t expected_hand_id,
                        HandCanFrame& response,
                        int timeout_ms) override;

private:
    std::array<int, 2> dev_indices_;
    std::array<int, 2> chn_indices_;
    bool close_on_destroy_;
    bool initialized_ = false;
};

#endif
