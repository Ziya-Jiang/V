#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "src/gripper/lingkong_can_protocol.h"

int main() {
    using namespace lingkong_can_protocol;

    const auto position = absolutePosition(50, -125.0);
    assert(position[0] == kAbsolutePosition);
    assert(position[1] == 0);
    assert(position[2] == 50);
    assert(position[3] == 0);
    assert(readLe32(position, 4) == -12500);

    const auto torque = writeTorqueLimit(200);
    assert(torque[0] == kWriteTorqueLimit);
    assert(torque[1] == kTorqueLimitIndex);
    assert(readLe16(torque, 4) == 200);

    Payload angle_reply{};
    angle_reply[0] = kReadAngle;
    const std::int64_t centidegrees = -80000;
    const auto raw = static_cast<std::uint64_t>(centidegrees);
    for (std::size_t index = 0; index < 7; ++index) {
        angle_reply[index + 1] =
            static_cast<std::uint8_t>((raw >> (8 * index)) & 0xFF);
    }
    assert(std::abs(decodeAngleDegrees(angle_reply) + 800.0) < 1e-9);

    std::cout << "Lingkong CAN protocol tests passed" << std::endl;
    return 0;
}
