#ifndef HKCLR_LITE_ARM_V2_HARDWARE_H
#define HKCLR_LITE_ARM_V2_HARDWARE_H

#include <string>
#include <vector>

namespace spark2 {

enum class MotorDriverKind {
    Tl,
    Rmd,
};

struct BusConfig {
    int channel{0};
    int baudrate{0};
};

struct MotorConfig {
    int id{0};
    bool enabled{false};
    double gear_ratio{1.0};
    int resolution{100};
    int sign{1};
    double motor_offset_rad{0.0};
    MotorDriverKind driver{MotorDriverKind::Rmd};
    bool has_brake{true};  /* true=有物理抱闸, false=无抱闸 (默认true兼容旧配置) */
};

struct TorqueSensorConfig {
    int id{0};
    bool enabled{false};
    int sign{1};
};

struct HardwareConfig {
    std::string schema_version;
    int num_joints{0};
    BusConfig can;
    BusConfig canfd;
    std::vector<MotorConfig> motors;
    std::vector<TorqueSensorConfig> torque_sensors;
};

} // namespace spark2

#endif
