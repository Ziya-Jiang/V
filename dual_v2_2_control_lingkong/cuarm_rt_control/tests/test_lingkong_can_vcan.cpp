#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "src/gripper/lingkong_can_gripper.h"
#include "src/gripper/lingkong_can_protocol.h"

namespace {

using lingkong_can_protocol::Payload;

class FakeLingkongDevice {
public:
    FakeLingkongDevice(std::string interface_name, int node_id)
        : can_id_(0x140U + static_cast<unsigned>(node_id)) {
        fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
        if (fd_ < 0) throw std::runtime_error("fake socket failed");
        ifreq request{};
        std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
            throw std::runtime_error("fake interface lookup failed");
        }
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = request.ifr_ifindex;
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            throw std::runtime_error("fake bind failed");
        }
        worker_ = std::thread(&FakeLingkongDevice::run, this);
    }

    ~FakeLingkongDevice() {
        stop_.store(true);
        if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
        if (worker_.joinable()) worker_.join();
        if (fd_ >= 0) ::close(fd_);
    }

private:
    void run() {
        while (!stop_.load()) {
            can_frame request{};
            if (::read(fd_, &request, sizeof(request)) != sizeof(request)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if ((request.can_id & CAN_SFF_MASK) != can_id_ || request.can_dlc != 8) {
                continue;
            }
            Payload response{};
            std::copy_n(request.data, 8, response.begin());
            switch (request.data[0]) {
                case lingkong_can_protocol::kReadAngle: {
                    const auto raw = static_cast<std::uint64_t>(
                        static_cast<std::int64_t>(std::llround(angle_deg_ * 100.0))
                    );
                    for (std::size_t index = 0; index < 7; ++index) {
                        response[index + 1] =
                            static_cast<std::uint8_t>((raw >> (8 * index)) & 0xFF);
                    }
                    break;
                }
                case lingkong_can_protocol::kReadStatus1:
                    response[1] = 25;
                    lingkong_can_protocol::writeLe16(response, 2, 2400);
                    lingkong_can_protocol::writeLe16(response, 4, 20);
                    response[6] = enabled_ ? 1 : 0;
                    response[7] = 0;
                    break;
                case lingkong_can_protocol::kReadStatus2:
                    response[1] = 25;
                    lingkong_can_protocol::writeLe16(response, 2, 10);
                    lingkong_can_protocol::writeLe16(response, 4, 0);
                    response[6] = 0x34;
                    response[7] = 0x12;
                    break;
                case lingkong_can_protocol::kReadTorqueLimit:
                    lingkong_can_protocol::writeLe16(response, 4, torque_limit_);
                    break;
                case lingkong_can_protocol::kWriteTorqueLimit:
                    torque_limit_ =
                        lingkong_can_protocol::readLe16(response, 4);
                    break;
                case lingkong_can_protocol::kEnable:
                    enabled_ = true;
                    break;
                case lingkong_can_protocol::kStop:
                case lingkong_can_protocol::kDisable:
                    enabled_ = false;
                    break;
                case lingkong_can_protocol::kAbsolutePosition:
                    angle_deg_ =
                        lingkong_can_protocol::readLe32(response, 4) / 100.0;
                    break;
                default:
                    continue;
            }
            can_frame reply{};
            reply.can_id = can_id_;
            reply.can_dlc = 8;
            std::copy(response.begin(), response.end(), reply.data);
            if (::write(fd_, &reply, sizeof(reply)) != sizeof(reply)) {
                return;
            }
        }
    }

    int fd_ = -1;
    unsigned can_id_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    double angle_deg_ = -125.0;
    int torque_limit_ = 100;
    bool enabled_ = false;
};

void waitForPanelValue(LingkongCanGripper& gripper, float expected) {
    for (int attempt = 0; attempt < 50; ++attempt) {
        std::vector<float> value(1);
        if (gripper.getPanelPos(value) == RetCode::kSuccess &&
            std::abs(value[0] - expected) < 0.02f) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("timed out waiting for simulated gripper feedback");
}

}  // namespace

int main(int argc, char** argv) {
    const std::string interface_name = argc > 1 ? argv[1] : "vcan0";
    FakeLingkongDevice device(interface_name, 8);

    LingkongCanGripperConfig config;
    config.interface_name = interface_name;
    config.node_id = 8;
    config.open_angle_deg = -800.0f;
    config.closed_angle_deg = -125.0f;
    config.calibrated = true;
    config.speed_dps = 50;
    config.torque_limit_raw = 200;
    config.telemetry_period_ms = 50;

    LingkongCanGripper gripper("test", config);
    assert(gripper.open() == RetCode::kSuccess);
    assert(gripper.setPanelCommand({0.0f}) == RetCode::kSuccess);
    waitForPanelValue(gripper, 0.0f);
    assert(gripper.setPanelCommand({1.0f}) == RetCode::kSuccess);
    waitForPanelValue(gripper, 1.0f);
    assert(gripper.close() == RetCode::kSuccess);

    config.calibrated = false;
    LingkongCanGripper locked_gripper("locked-test", config);
    assert(locked_gripper.open() == RetCode::kSuccess);
    assert(locked_gripper.setPanelCommand({0.0f}) == RetCode::kInvalidInput);
    assert(locked_gripper.close() == RetCode::kSuccess);

    std::cout << "Lingkong vcan integration test passed" << std::endl;
    return 0;
}
