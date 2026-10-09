#include "spark2_config_loader.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <sstream>

namespace spark2 {
namespace {

void append_err(std::string *err, const std::string &msg)
{
    if (!err) {
        return;
    }
    if (!err->empty()) {
        *err += "; ";
    }
    *err += msg;
}

bool read_bus(const YAML::Node &bus, BusConfig *out, std::string *err, const char *path_label)
{
    if (!bus || !bus.IsMap()) {
        append_err(err, std::string("missing or invalid bus map: ") + path_label);
        return false;
    }
    if (!bus["channel"] || !bus["channel"].IsScalar()) {
        append_err(err, std::string("missing hardware.") + path_label + ".channel");
        return false;
    }
    if (!bus["baudrate"] || !bus["baudrate"].IsScalar()) {
        append_err(err, std::string("missing hardware.") + path_label + ".baudrate");
        return false;
    }
    out->channel = bus["channel"].as<int>();
    out->baudrate = bus["baudrate"].as<int>();
    return true;
}

bool parse_driver_kind(const std::string &s, MotorDriverKind *out)
{
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (t == "tl") {
        *out = MotorDriverKind::Tl;
        return true;
    }
    if (t == "rmd") {
        *out = MotorDriverKind::Rmd;
        return true;
    }
    return false;
}

template <typename T>
bool read_scalar_seq(const YAML::Node &n, std::vector<T> *out, std::string *err, const char *field)
{
    if (!n || !n.IsSequence()) {
        append_err(err, std::string("missing or invalid sequence: ") + field);
        return false;
    }
    out->clear();
    out->reserve(n.size());
    for (std::size_t i = 0; i < n.size(); ++i) {
        if (!n[i].IsScalar()) {
            append_err(err, std::string("non-scalar entry in ") + field);
            return false;
        }
        out->push_back(n[i].as<T>());
    }
    return true;
}

bool read_bool_seq_from_ints(const YAML::Node &n, std::vector<bool> *out, std::string *err, const char *field)
{
    if (!n || !n.IsSequence()) {
        append_err(err, std::string("missing or invalid sequence: ") + field);
        return false;
    }
    out->clear();
    out->reserve(n.size());
    for (std::size_t i = 0; i < n.size(); ++i) {
        if (!n[i].IsScalar()) {
            append_err(err, std::string("non-scalar entry in ") + field);
            return false;
        }
        const int v = n[i].as<int>();
        if (v != 0 && v != 1) {
            append_err(err, std::string("hardware.motor.enabled must be 0 or 1 per axis"));
            return false;
        }
        out->push_back(v != 0);
    }
    return true;
}

bool read_driver_seq(const YAML::Node &n, std::vector<MotorDriverKind> *out, std::string *err)
{
    if (!n || !n.IsSequence()) {
        append_err(err, "missing or invalid sequence: hardware.motor.motor_type");
        return false;
    }
    out->clear();
    out->reserve(n.size());
    for (std::size_t i = 0; i < n.size(); ++i) {
        if (!n[i].IsScalar()) {
            append_err(err, "non-scalar entry in hardware.motor.motor_type");
            return false;
        }
        MotorDriverKind k{};
        if (!parse_driver_kind(n[i].as<std::string>(), &k)) {
            std::ostringstream os;
            os << "unknown motor_type at index " << i << ": " << n[i].as<std::string>();
            append_err(err, os.str());
            return false;
        }
        out->push_back(k);
    }
    return true;
}

bool same_length(std::size_t n, std::size_t m, const char *a, const char *b, std::string *err)
{
    if (n != m) {
        std::ostringstream os;
        os << "length mismatch: " << a << " (" << n << ") vs " << b << " (" << m << ")";
        append_err(err, os.str());
        return false;
    }
    return true;
}

} // namespace

bool load_hardware_yaml(const std::string &path, HardwareConfig *out, std::string *err)
{
    if (!out) {
        append_err(err, "out is null");
        return false;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception &e) {
        append_err(err, std::string("YAML load failed: ") + e.what());
        return false;
    }

    if (!root.IsMap()) {
        append_err(err, "root must be a map");
        return false;
    }

    HardwareConfig cfg{};
    if (root["version"]) {
        cfg.schema_version = root["version"].as<std::string>("");
    }

    const YAML::Node hw = root["hardware"];
    if (!hw || !hw.IsMap()) {
        append_err(err, "missing top-level 'hardware' map");
        return false;
    }

    if (!read_bus(hw["can"], &cfg.can, err, "can")) {
        return false;
    }
    if (!read_bus(hw["canfd"], &cfg.canfd, err, "canfd")) {
        return false;
    }

    const YAML::Node motor = hw["motor"];
    if (!motor || !motor.IsMap()) {
        append_err(err, "missing hardware.motor map");
        return false;
    }

    std::vector<int> ids;
    std::vector<bool> enabled;
    std::vector<double> gears;
    std::vector<int> resolutions;
    std::vector<int> signs;
    std::vector<double> offsets;
    std::vector<MotorDriverKind> types;

    if (!read_scalar_seq(motor["ids"], &ids, err, "hardware.motor.ids")) {
        return false;
    }
    if (ids.empty()) {
        append_err(err, "hardware.motor.ids is empty");
        return false;
    }

    if (!read_bool_seq_from_ints(motor["enabled"], &enabled, err, "hardware.motor.enabled")) {
        return false;
    }
    if (!read_scalar_seq(motor["gear_ratios"], &gears, err, "hardware.motor.gear_ratios")) {
        return false;
    }

    if (motor["resolution"]) {
        if (!read_scalar_seq(motor["resolution"], &resolutions, err, "hardware.motor.resolution")) {
            return false;
        }
    } else {
        resolutions.assign(ids.size(), 100);
    }

    if (motor["sign"]) {
        if (!read_scalar_seq(motor["sign"], &signs, err, "hardware.motor.sign")) {
            return false;
        }
    } else {
        signs.assign(ids.size(), 1);
    }

    if (motor["motor_offset"]) {
        if (!read_scalar_seq(motor["motor_offset"], &offsets, err, "hardware.motor.motor_offset")) {
            return false;
        }
    } else {
        offsets.assign(ids.size(), 0.0);
    }

    if (!read_driver_seq(motor["motor_type"], &types, err)) {
        return false;
    }

    const std::size_t n = ids.size();
    if (!same_length(n, enabled.size(), "ids", "enabled", err)) {
        return false;
    }
    if (!same_length(n, gears.size(), "ids", "gear_ratios", err)) {
        return false;
    }
    if (!same_length(n, resolutions.size(), "ids", "resolution", err)) {
        return false;
    }
    if (!same_length(n, signs.size(), "ids", "sign", err)) {
        return false;
    }
    if (!same_length(n, offsets.size(), "ids", "motor_offset", err)) {
        return false;
    }
    if (!same_length(n, types.size(), "ids", "motor_type", err)) {
        return false;
    }

    if (root["num_joints"]) {
        cfg.num_joints = root["num_joints"].as<int>();
        if (cfg.num_joints < 0) {
            append_err(err, "num_joints must be non-negative");
            return false;
        }
        if (static_cast<std::size_t>(cfg.num_joints) != n) {
            append_err(err, "num_joints does not match hardware.motor.ids length");
            return false;
        }
    } else {
        cfg.num_joints = static_cast<int>(n);
    }

    cfg.motors.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        cfg.motors[i].id = ids[i];
        cfg.motors[i].enabled = enabled[i];
        cfg.motors[i].gear_ratio = gears[i];
        cfg.motors[i].resolution = resolutions[i];
        cfg.motors[i].sign = signs[i];
        if (cfg.motors[i].sign != 1 && cfg.motors[i].sign != -1) {
            append_err(err, "hardware.motor.sign entries must be 1 or -1");
            return false;
        }
        cfg.motors[i].motor_offset_rad = offsets[i];
        cfg.motors[i].driver = types[i];
    }

    /* has_brake (可选字段, 默认 true 兼容旧配置) */
    if (motor["has_brake"]) {
        std::vector<bool> brakes;
        if (!read_bool_seq_from_ints(motor["has_brake"], &brakes, err, "hardware.motor.has_brake")) {
            return false;
        }
        if (!same_length(n, brakes.size(), "ids", "has_brake", err)) {
            return false;
        }
        for (std::size_t i = 0; i < n; ++i) {
            cfg.motors[i].has_brake = brakes[i];
        }
    }

    const YAML::Node ts = hw["torque_sensor"];
    if (ts && ts.IsMap()) {
        std::vector<int> ts_ids;
        std::vector<bool> ts_en;
        std::vector<int> ts_sign;
        if (!read_scalar_seq(ts["ids"], &ts_ids, err, "hardware.torque_sensor.ids")) {
            return false;
        }
        if (!read_bool_seq_from_ints(ts["enabled"], &ts_en, err, "hardware.torque_sensor.enabled")) {
            return false;
        }
        if (!read_scalar_seq(ts["sign"], &ts_sign, err, "hardware.torque_sensor.sign")) {
            return false;
        }
        const std::size_t m = ts_ids.size();
        if (!same_length(m, ts_en.size(), "torque_sensor.ids", "enabled", err)) {
            return false;
        }
        if (!same_length(m, ts_sign.size(), "torque_sensor.ids", "sign", err)) {
            return false;
        }
        cfg.torque_sensors.resize(m);
        for (std::size_t i = 0; i < m; ++i) {
            cfg.torque_sensors[i].id = ts_ids[i];
            cfg.torque_sensors[i].enabled = ts_en[i];
            cfg.torque_sensors[i].sign = ts_sign[i];
            if (cfg.torque_sensors[i].sign != 1 && cfg.torque_sensors[i].sign != -1) {
                append_err(err, "hardware.torque_sensor.sign entries must be 1 or -1");
                return false;
            }
        }
    }

    *out = std::move(cfg);
    return true;
}

} // namespace spark2
