#include "lingkong_can_gripper.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <poll.h>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "lingkong_can_protocol.h"

namespace {

using Payload = lingkong_can_protocol::Payload;

std::int16_t checkedInt16(int value, const char* label) {
    if (value < std::numeric_limits<std::int16_t>::min() ||
        value > std::numeric_limits<std::int16_t>::max()) {
        throw std::invalid_argument(std::string(label) + " is out of int16 range");
    }
    return static_cast<std::int16_t>(value);
}

}  // namespace

void LingkongCanGripperConfig::validate() const {
    const bool finite_angles =
        std::isfinite(open_angle_deg) && std::isfinite(closed_angle_deg) &&
        std::isfinite(min_angle_deg) && std::isfinite(max_angle_deg);
    if (interface_name.empty()) {
        throw std::invalid_argument("Lingkong CAN interface is required");
    }
    if (node_id < 1 || node_id > 32) {
        throw std::invalid_argument("Lingkong node_id must be in 1..32");
    }
    if (!finite_angles || min_angle_deg >= max_angle_deg ||
        open_angle_deg < min_angle_deg || open_angle_deg > max_angle_deg ||
        closed_angle_deg < min_angle_deg || closed_angle_deg > max_angle_deg ||
        open_angle_deg == closed_angle_deg) {
        throw std::invalid_argument("Lingkong angle configuration is invalid");
    }
    if (max_speed_dps < 1 || max_speed_dps > 200 ||
        speed_dps < 1 || speed_dps > max_speed_dps) {
        throw std::invalid_argument("Lingkong speed exceeds the 200 dps safety cap");
    }
    if (max_torque_limit_raw < 1 || max_torque_limit_raw > 750 ||
        torque_limit_raw < 1 || torque_limit_raw > max_torque_limit_raw) {
        throw std::invalid_argument(
            "Lingkong torque limit exceeds the 750 raw safety cap"
        );
    }
    if (max_temperature_c <= 0 || !std::isfinite(min_voltage_v) ||
        !std::isfinite(max_voltage_v) || min_voltage_v <= 0.0f ||
        min_voltage_v >= max_voltage_v) {
        throw std::invalid_argument("Lingkong telemetry safety limits are invalid");
    }
    if (response_timeout_ms < 10 || response_timeout_ms > 1000 ||
        telemetry_period_ms < 50 || telemetry_period_ms > 2000 ||
        keepalive_ms < 50 || keepalive_ms > 5000 ||
        command_watchdog_ms < 100 || command_watchdog_ms > 30000) {
        throw std::invalid_argument("Lingkong timing configuration is invalid");
    }
}

LingkongCanGripper::LingkongCanGripper(
    const std::string& name,
    LingkongCanGripperConfig config
)
    : AbstractGripper(
          name,
          1,
          // Direct normalized command 1.0 remains the startup-open command.
          // Panel command conversion below maps panel 0=open and 1=closed.
          {{config.closed_angle_deg, config.open_angle_deg}}
      ),
      config_(std::move(config)),
      can_id_(0x140U + static_cast<std::uint32_t>(config_.node_id)) {
    config_.validate();
    avoid_duplicate_command_ = false;
    round_command_digits_ = -1;
}

LingkongCanGripper::~LingkongCanGripper() {
    close();
}

void LingkongCanGripper::openSocket() {
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd_ < 0) {
        throw std::runtime_error(
            "Cannot create Lingkong CAN socket: " + std::string(std::strerror(errno))
        );
    }

    ifreq request{};
    std::strncpy(
        request.ifr_name,
        config_.interface_name.c_str(),
        IFNAMSIZ - 1
    );
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
        closeSocket();
        throw std::runtime_error(
            "Cannot resolve Lingkong CAN interface " + config_.interface_name
        );
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(
            socket_fd_,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address)
        ) < 0) {
        closeSocket();
        throw std::runtime_error(
            "Cannot bind Lingkong CAN interface " + config_.interface_name
        );
    }

    can_filter filter{};
    filter.can_id = can_id_;
    filter.can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    if (::setsockopt(
            socket_fd_,
            SOL_CAN_RAW,
            CAN_RAW_FILTER,
            &filter,
            sizeof(filter)
        ) < 0) {
        closeSocket();
        throw std::runtime_error("Cannot install Lingkong CAN receive filter");
    }
}

void LingkongCanGripper::closeSocket() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

Payload LingkongCanGripper::request(
    const Payload& payload,
    int timeout_ms
) {
    if (socket_fd_ < 0) {
        throw std::runtime_error("Lingkong CAN socket is closed");
    }
    const int wait_ms =
        timeout_ms > 0 ? timeout_ms : config_.response_timeout_ms;

    can_frame stale{};
    while (::read(socket_fd_, &stale, sizeof(stale)) == sizeof(stale)) {
    }

    can_frame outgoing{};
    outgoing.can_id = can_id_;
    outgoing.can_dlc = 8;
    std::copy(payload.begin(), payload.end(), outgoing.data);
    if (::write(socket_fd_, &outgoing, sizeof(outgoing)) != sizeof(outgoing)) {
        throw std::runtime_error(
            "Lingkong CAN write failed: " + std::string(std::strerror(errno))
        );
    }

    const auto deadline = Clock::now() + std::chrono::milliseconds(wait_ms);
    while (Clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - Clock::now()
        ).count();
        pollfd descriptor{socket_fd_, POLLIN, 0};
        const int result = ::poll(
            &descriptor,
            1,
            std::max(1, static_cast<int>(remaining))
        );
        if (result < 0 && errno != EINTR) {
            throw std::runtime_error(
                "Lingkong CAN poll failed: " + std::string(std::strerror(errno))
            );
        }
        if (result <= 0) {
            continue;
        }
        can_frame incoming{};
        if (::read(socket_fd_, &incoming, sizeof(incoming)) != sizeof(incoming)) {
            continue;
        }
        if ((incoming.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
            (incoming.can_id & CAN_SFF_MASK) != can_id_ ||
            incoming.can_dlc != 8 ||
            incoming.data[0] != payload[0]) {
            continue;
        }
        Payload response{};
        std::copy_n(incoming.data, 8, response.begin());
        return response;
    }
    throw std::runtime_error(
        "Lingkong node " + std::to_string(config_.node_id) +
        " command 0x" +
        [] (std::uint8_t command) {
            constexpr char digits[] = "0123456789ABCDEF";
            std::string text(2, '0');
            text[0] = digits[(command >> 4) & 0xF];
            text[1] = digits[command & 0xF];
            return text;
        }(payload[0]) +
        " reply timed out"
    );
}

LingkongCanGripper::State LingkongCanGripper::readState() {
    const auto angle_reply = request(
        lingkong_can_protocol::query(lingkong_can_protocol::kReadAngle)
    );
    const auto status1 = request(
        lingkong_can_protocol::query(lingkong_can_protocol::kReadStatus1)
    );
    const auto status2 = request(
        lingkong_can_protocol::query(lingkong_can_protocol::kReadStatus2)
    );

    State state;
    state.angle_deg = static_cast<float>(
        lingkong_can_protocol::decodeAngleDegrees(angle_reply)
    );
    state.temperature_c = static_cast<std::int8_t>(status1[1]);
    state.voltage_v =
        static_cast<float>(lingkong_can_protocol::readLe16(status1, 2)) / 100.0f;
    state.current_a =
        static_cast<float>(lingkong_can_protocol::readLe16(status1, 4)) / 100.0f;
    state.motor_state = status1[6];
    state.error_state = status1[7];
    state.torque_raw = lingkong_can_protocol::readLe16(status2, 2);
    state.speed_dps = lingkong_can_protocol::readLe16(status2, 4);
    state.encoder_raw =
        static_cast<std::uint16_t>(status2[6]) |
        (static_cast<std::uint16_t>(status2[7]) << 8);

    if (state.error_state != 0) {
        throw std::runtime_error(
            "Lingkong device error " + std::to_string(state.error_state)
        );
    }
    if (state.temperature_c > config_.max_temperature_c) {
        throw std::runtime_error("Lingkong over-temperature");
    }
    if (state.voltage_v < config_.min_voltage_v ||
        state.voltage_v > config_.max_voltage_v) {
        throw std::runtime_error("Lingkong voltage is outside configured range");
    }
    if (!commandIsInPhysicalRange(state.angle_deg)) {
        throw std::runtime_error("Lingkong angle is outside configured bounds");
    }
    return state;
}

int LingkongCanGripper::readTorqueLimit() {
    const auto response = request(lingkong_can_protocol::readTorqueLimit());
    return lingkong_can_protocol::readLe16(response, 4);
}

void LingkongCanGripper::writeTorqueLimit(int limit) {
    const auto response = request(
        lingkong_can_protocol::writeTorqueLimit(
            checkedInt16(limit, "Lingkong torque limit")
        )
    );
    if (lingkong_can_protocol::readLe16(response, 4) != limit) {
        throw std::runtime_error("Lingkong torque-limit verification failed");
    }
    active_torque_limit_ = limit;
}

void LingkongCanGripper::enableMotor() {
    if (motor_enabled_) {
        return;
    }
    if (active_torque_limit_ != config_.torque_limit_raw) {
        writeTorqueLimit(config_.torque_limit_raw);
        torque_limit_written_ = true;
    }
    request(lingkong_can_protocol::query(lingkong_can_protocol::kEnable));
    motor_enabled_ = true;
}

void LingkongCanGripper::stopAndDisableNoThrow() {
    if (!motor_enabled_) {
        return;
    }
    try {
        request(
            lingkong_can_protocol::query(lingkong_can_protocol::kStop),
            std::min(200, config_.response_timeout_ms)
        );
    } catch (const std::exception&) {
    }
    try {
        request(
            lingkong_can_protocol::query(lingkong_can_protocol::kDisable),
            std::min(200, config_.response_timeout_ms)
        );
    } catch (const std::exception&) {
    }
    motor_enabled_ = false;
    last_sent_angle_valid_ = false;
}

void LingkongCanGripper::restoreTorqueLimitNoThrow() {
    if (!original_torque_limit_valid_ || !torque_limit_written_) {
        return;
    }
    try {
        writeTorqueLimit(original_torque_limit_);
    } catch (const std::exception& error) {
        std::cerr << "[Lingkong] Failed to restore torque limit: "
                  << error.what() << std::endl;
    }
}

RetCode LingkongCanGripper::open() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (opened_) {
        return RetCode::kSuccess;
    }
    try {
        config_.validate();
        openSocket();
        state_ = readState();
        state_valid_ = true;
        original_torque_limit_ = readTorqueLimit();
        active_torque_limit_ = original_torque_limit_;
        original_torque_limit_valid_ = true;
        stop_requested_.store(false);
        opened_ = true;
        worker_ = std::thread(&LingkongCanGripper::workerLoop, this);
        return RetCode::kSuccess;
    } catch (const std::exception& error) {
        faulted_ = true;
        fault_message_ = error.what();
        closeSocket();
        return RetCode::kFailed;
    }
}

RetCode LingkongCanGripper::close() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!opened_ && socket_fd_ < 0) {
            return RetCode::kSuccess;
        }
        stop_requested_.store(true);
        command_cv_.notify_all();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    stopAndDisableNoThrow();
    restoreTorqueLimitNoThrow();
    closeSocket();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        opened_ = false;
        target_valid_ = false;
    }
    return RetCode::kSuccess;
}

RetCode LingkongCanGripper::setPosition(const std::vector<float>& value) {
    if (value.size() != 1 || !std::isfinite(value[0])) {
        return RetCode::kInvalidInput;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!opened_ || faulted_) {
        return RetCode::kFailed;
    }
    if (!config_.calibrated || !commandIsInPhysicalRange(value[0])) {
        return RetCode::kInvalidInput;
    }
    target_angle_deg_ = value[0];
    target_updated_at_ = Clock::now();
    target_valid_ = true;
    last_set_command_[0] = value[0];
    command_cv_.notify_all();
    return RetCode::kSuccess;
}

RetCode LingkongCanGripper::getPosition(std::vector<float>& value) {
    if (value.size() != 1) {
        return RetCode::kInvalidInput;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!opened_ || faulted_ || !state_valid_) {
        return RetCode::kFailed;
    }
    value[0] = state_.angle_deg;
    return RetCode::kSuccess;
}

std::vector<float> LingkongCanGripper::convertPanelHardware(
    const std::vector<float>& value
) {
    if (value.size() != 1 || !std::isfinite(value[0])) {
        return {};
    }
    return {1.0f - value[0]};
}

bool LingkongCanGripper::hasFault() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return faulted_;
}

std::string LingkongCanGripper::faultMessage() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return fault_message_;
}

bool LingkongCanGripper::commandIsInPhysicalRange(float angle_deg) const {
    return std::isfinite(angle_deg) &&
           angle_deg >= config_.min_angle_deg &&
           angle_deg <= config_.max_angle_deg;
}

void LingkongCanGripper::setFault(const std::string& message) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        faulted_ = true;
        fault_message_ = message;
        target_valid_ = false;
    }
    std::cerr << "[Lingkong] " << message << std::endl;
}

void LingkongCanGripper::workerLoop() {
    auto next_telemetry = Clock::now();
    try {
        while (!stop_requested_.load()) {
            bool target_valid = false;
            float target_angle = 0.0f;
            Clock::time_point target_updated;
            {
                std::unique_lock<std::mutex> lock(state_mutex_);
                command_cv_.wait_for(
                    lock,
                    std::chrono::milliseconds(20),
                    [this] { return stop_requested_.load(); }
                );
                if (stop_requested_.load()) {
                    break;
                }
                target_valid = target_valid_;
                target_angle = target_angle_deg_;
                target_updated = target_updated_at_;
            }

            const auto now = Clock::now();
            if (target_valid &&
                now - target_updated >
                    std::chrono::milliseconds(config_.command_watchdog_ms)) {
                stopAndDisableNoThrow();
                std::lock_guard<std::mutex> lock(state_mutex_);
                target_valid_ = false;
                target_valid = false;
            }

            if (target_valid) {
                const bool changed =
                    !last_sent_angle_valid_ ||
                    std::abs(target_angle - last_sent_angle_deg_) > 0.01f;
                const bool keepalive_due =
                    !last_sent_angle_valid_ ||
                    now - last_command_sent_at_ >=
                        std::chrono::milliseconds(config_.keepalive_ms);
                if (changed || keepalive_due) {
                    enableMotor();
                    request(lingkong_can_protocol::absolutePosition(
                        static_cast<std::uint16_t>(config_.speed_dps),
                        target_angle
                    ));
                    last_sent_angle_deg_ = target_angle;
                    last_sent_angle_valid_ = true;
                    last_command_sent_at_ = Clock::now();
                }
            }

            if (Clock::now() >= next_telemetry) {
                const State state = readState();
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    state_ = state;
                    state_valid_ = true;
                }
                next_telemetry =
                    Clock::now() +
                    std::chrono::milliseconds(config_.telemetry_period_ms);
            }
        }
    } catch (const std::exception& error) {
        setFault(error.what());
    }
    stopAndDisableNoThrow();
}
