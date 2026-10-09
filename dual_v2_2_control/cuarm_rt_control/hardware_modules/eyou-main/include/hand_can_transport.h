#ifndef HAND_CAN_TRANSPORT_H
#define HAND_CAN_TRANSPORT_H

#include <array>
#include <cstdint>

constexpr int HAND_CAN_SUCCESS = 0;
constexpr int HAND_CAN_FAILED_RECEIVE = -2;

struct HandCanFrame {
    uint32_t id = 0;
    bool is_extended = true;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

class IHandCanTransport {
public:
    virtual ~IHandCanTransport() = default;

    virtual int request(int hand_idx,
                        uint32_t expected_hand_id,
                        const HandCanFrame& request,
                        HandCanFrame& response,
                        int timeout_ms) = 0;

    virtual int waitForResponse(int hand_idx,
                                uint32_t expected_hand_id,
                                HandCanFrame& response,
                                int timeout_ms) = 0;
};

class CanableHandTransport : public IHandCanTransport {
public:
    explicit CanableHandTransport(const int* hand_ids);
    ~CanableHandTransport() override;

    int request(int hand_idx,
                uint32_t expected_hand_id,
                const HandCanFrame& request,
                HandCanFrame& response,
                int timeout_ms) override;

    int waitForResponse(int hand_idx,
                        uint32_t expected_hand_id,
                        HandCanFrame& response,
                        int timeout_ms) override;

    // Sends one frame and returns the first reply. When filter_hand_id is false,
    // any received frame is returned so an unknown hand ID can be observed.
    int exchange(const HandCanFrame& request,
                 HandCanFrame& response,
                 int timeout_ms,
                 bool filter_hand_id,
                 uint32_t expected_hand_id);

private:
    int device_index_ = -1;
};

#endif
