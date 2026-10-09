#ifndef LINGKONG_CAN_GRIPPER_H
#define LINGKONG_CAN_GRIPPER_H

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "src/gripper/abstract_gripper.h"

struct LingkongCanGripperConfig {
    std::string interface_name;
    int node_id = 0;
    float open_angle_deg = 0.0f;
    float closed_angle_deg = 0.0f;
    bool calibrated = false;
    int speed_dps = 50;
    int torque_limit_raw = 200;
    int max_speed_dps = 200;
    int max_torque_limit_raw = 750;
    float min_angle_deg = -800.0f;
    float max_angle_deg = 0.0f;
    int max_temperature_c = 60;
    float min_voltage_v = 18.0f;
    float max_voltage_v = 30.0f;
    int response_timeout_ms = 300;
    int telemetry_period_ms = 200;
    int keepalive_ms = 500;
    int command_watchdog_ms = 1000;

    void validate() const;
};

class LingkongCanGripper final : public AbstractGripper {
public:
    explicit LingkongCanGripper(
        const std::string& name,
        LingkongCanGripperConfig config
    );
    ~LingkongCanGripper();

    RetCode open() override;
    RetCode close() override;
    RetCode setPosition(const std::vector<float>& value) override;
    RetCode getPosition(std::vector<float>& value) override;
    std::vector<float> convertPanelHardware(
        const std::vector<float>& value
    ) override;

    bool hasFault() const;
    std::string faultMessage() const;

private:
    struct State {
        float angle_deg = 0.0f;
        int temperature_c = 0;
        float voltage_v = 0.0f;
        float current_a = 0.0f;
        int torque_raw = 0;
        int speed_dps = 0;
        int encoder_raw = 0;
        int motor_state = 0;
        int error_state = 0;
    };

    using Clock = std::chrono::steady_clock;

    void openSocket();
    void closeSocket();
    std::array<std::uint8_t, 8> request(
        const std::array<std::uint8_t, 8>& payload,
        int timeout_ms = -1
    );
    State readState();
    int readTorqueLimit();
    void writeTorqueLimit(int limit);
    void enableMotor();
    void stopAndDisableNoThrow();
    void restoreTorqueLimitNoThrow();
    void workerLoop();
    void setFault(const std::string& message);
    bool commandIsInPhysicalRange(float angle_deg) const;

    LingkongCanGripperConfig config_;
    int socket_fd_ = -1;
    std::uint32_t can_id_ = 0;
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;

    mutable std::mutex state_mutex_;
    std::condition_variable command_cv_;
    State state_;
    bool state_valid_ = false;
    bool target_valid_ = false;
    float target_angle_deg_ = 0.0f;
    Clock::time_point target_updated_at_{};
    Clock::time_point last_command_sent_at_{};
    float last_sent_angle_deg_ = 0.0f;
    bool last_sent_angle_valid_ = false;
    bool motor_enabled_ = false;
    bool opened_ = false;
    bool faulted_ = false;
    std::string fault_message_;
    int original_torque_limit_ = 0;
    int active_torque_limit_ = 0;
    bool original_torque_limit_valid_ = false;
    bool torque_limit_written_ = false;
};

#endif
