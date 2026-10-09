#ifndef LINGKONG_CAN_PROTOCOL_H
#define LINGKONG_CAN_PROTOCOL_H

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace lingkong_can_protocol {

using Payload = std::array<std::uint8_t, 8>;

constexpr std::uint8_t kReadAngle = 0x92;
constexpr std::uint8_t kReadStatus1 = 0x9A;
constexpr std::uint8_t kReadStatus2 = 0x9C;
constexpr std::uint8_t kReadTorqueLimit = 0xC0;
constexpr std::uint8_t kWriteTorqueLimit = 0xC1;
constexpr std::uint8_t kEnable = 0x88;
constexpr std::uint8_t kStop = 0x81;
constexpr std::uint8_t kDisable = 0x80;
constexpr std::uint8_t kAbsolutePosition = 0xA4;
constexpr std::uint8_t kTorqueLimitIndex = 0x1E;

inline Payload query(std::uint8_t command) {
    Payload payload{};
    payload[0] = command;
    return payload;
}

inline void writeLe16(Payload& payload, std::size_t offset, std::int16_t value) {
    const auto raw = static_cast<std::uint16_t>(value);
    payload[offset] = static_cast<std::uint8_t>(raw & 0xFF);
    payload[offset + 1] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
}

inline std::int16_t readLe16(const Payload& payload, std::size_t offset) {
    const auto raw = static_cast<std::uint16_t>(payload[offset]) |
                     (static_cast<std::uint16_t>(payload[offset + 1]) << 8);
    return static_cast<std::int16_t>(raw);
}

inline void writeLe32(Payload& payload, std::size_t offset, std::int32_t value) {
    const auto raw = static_cast<std::uint32_t>(value);
    for (std::size_t index = 0; index < 4; ++index) {
        payload[offset + index] =
            static_cast<std::uint8_t>((raw >> (8 * index)) & 0xFF);
    }
}

inline std::int32_t readLe32(const Payload& payload, std::size_t offset) {
    std::uint32_t raw = 0;
    for (std::size_t index = 0; index < 4; ++index) {
        raw |= static_cast<std::uint32_t>(payload[offset + index])
               << (8 * index);
    }
    return static_cast<std::int32_t>(raw);
}

inline Payload readTorqueLimit() {
    auto payload = query(kReadTorqueLimit);
    payload[1] = kTorqueLimitIndex;
    return payload;
}

inline Payload writeTorqueLimit(std::int16_t limit) {
    auto payload = query(kWriteTorqueLimit);
    payload[1] = kTorqueLimitIndex;
    writeLe16(payload, 4, limit);
    return payload;
}

inline Payload absolutePosition(std::uint16_t speed_dps, double angle_deg) {
    if (!std::isfinite(angle_deg)) {
        throw std::invalid_argument("Lingkong angle must be finite");
    }
    auto payload = query(kAbsolutePosition);
    payload[2] = static_cast<std::uint8_t>(speed_dps & 0xFF);
    payload[3] = static_cast<std::uint8_t>((speed_dps >> 8) & 0xFF);
    const auto centidegrees = static_cast<std::int32_t>(
        std::llround(angle_deg * 100.0)
    );
    writeLe32(payload, 4, centidegrees);
    return payload;
}

inline double decodeAngleDegrees(const Payload& payload) {
    std::uint64_t raw = 0;
    for (std::size_t index = 0; index < 7; ++index) {
        raw |= static_cast<std::uint64_t>(payload[index + 1]) << (8 * index);
    }
    if ((raw & (std::uint64_t{1} << 55)) != 0) {
        raw |= 0xFF00000000000000ULL;
    }
    return static_cast<double>(static_cast<std::int64_t>(raw)) / 100.0;
}

}  // namespace lingkong_can_protocol

#endif
