#include "inspire_hand.h"
#include <cassert>
#include <initializer_list>
#include <memory>
#include <vector>

class FakeHandCanTransport : public IHandCanTransport {
public:
    std::vector<HandCanFrame> requests;

    int request(int hand_idx,
                uint32_t expected_hand_id,
                const HandCanFrame& request,
                HandCanFrame& response,
                int timeout_ms) override {
        (void)hand_idx;
        (void)expected_hand_id;
        (void)timeout_ms;
        requests.push_back(request);

        const uint32_t reg_addr = (request.id >> 14) & 0xFFF;
        if (reg_addr == 0x060A) {
            fillResponse(response, {10, 20, 30, 40});
        } else if (reg_addr == 0x0612) {
            fillResponse(response, {50, 60});
        }
        return HAND_CAN_SUCCESS;
    }

    int waitForResponse(int hand_idx,
                        uint32_t expected_hand_id,
                        HandCanFrame& response,
                        int timeout_ms) override {
        (void)hand_idx;
        (void)expected_hand_id;
        (void)response;
        (void)timeout_ms;
        return HAND_CAN_SUCCESS;
    }

private:
    void fillResponse(HandCanFrame& response, std::initializer_list<int16_t> values) {
        response.dlc = static_cast<uint8_t>(values.size() * 2);
        int idx = 0;
        for (int16_t value : values) {
            response.data[idx++] = value & 0xFF;
            response.data[idx++] = (value >> 8) & 0xFF;
        }
    }
};

int main() {
    int hand_ids[2] = {2, 1};
    auto transport = std::make_shared<FakeHandCanTransport>();
    InspireHand hand(hand_ids, transport);

    int16_t angles[6] = {1000, 900, 800, 700, 600, 500};
    hand.setAngles(0, angles);

    assert(transport->requests.size() == 2);
    assert(((transport->requests[0].id >> 14) & 0xFFF) == 0x05CE);
    assert(((transport->requests[1].id >> 14) & 0xFFF) == 0x05D6);
    assert(transport->requests[0].dlc == 8);
    assert(transport->requests[1].dlc == 4);
    assert(transport->requests[0].data[0] == 0xE8);
    assert(transport->requests[0].data[1] == 0x03);
    assert(transport->requests[1].data[0] == 0x58);
    assert(transport->requests[1].data[1] == 0x02);

    assert(hand.getAngles(0) == 0);
    assert(hand.joint_angle_[0][0] == 10);
    assert(hand.joint_angle_[0][1] == 20);
    assert(hand.joint_angle_[0][2] == 30);
    assert(hand.joint_angle_[0][3] == 40);
    assert(hand.joint_angle_[0][4] == 50);
    assert(hand.joint_angle_[0][5] == 60);
    return 0;
}
