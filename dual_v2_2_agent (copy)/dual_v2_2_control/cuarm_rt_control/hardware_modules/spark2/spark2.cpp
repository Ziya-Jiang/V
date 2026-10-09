#include "spark2.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

#include "hardware_modules/cxt_can/linux/controlcan.h"
#include "hardware_modules/joint_interface/joint.h"
#include "robot_platform_utils/cpp/include/time_utils.h"
#include "src/file_utils.h"

namespace {

bool read_joint_position(JointActuatorBase *joint)
{
    if (auto *rmd = dynamic_cast<RmdCanActuator *>(joint)) {
        return rmd->read_absolute_position() == 0;
    }
    return joint->read_status() == 0;
}

bool refresh_joint_feedback(JointActuatorBase *joint, bool need_pos, bool need_vel_tor)
{
    if (auto *rmd = dynamic_cast<RmdCanActuator *>(joint)) {
        if (need_pos && rmd->read_absolute_position() != 0) {
            return false;
        }
        return true;
    }
    if (need_pos || need_vel_tor) {
        return joint->read_status() == 0;
    }
    return true;
}

}  // namespace

std::string Spark2::resolve_config_path(const std::string &config_path)
{
    if (!config_path.empty() && config_path.front() == '/') {
        return config_path;
    }
    return get_absolute_path(config_path);
}

int Spark2::load_joint_count_or_throw(const std::string &config_path)
{
    spark2::HardwareConfig hw{};
    std::string err;
    const std::string abs_config = resolve_config_path(config_path);
    if (!spark2::load_hardware_yaml(abs_config, &hw, &err)) {
        throw std::runtime_error("Spark2: failed to load hardware YAML '" + abs_config + "': " + err);
    }
    return hw.num_joints;
}

int Spark2::logical_channel_index(const int *chn_idx, int chn_num, int phys_channel)
{
    for (int i = 0; i < chn_num; ++i) {
        if (chn_idx[i] == phys_channel) {
            return i;
        }
    }
    return -1;
}

int Spark2::phys_channel_for_driver(const spark2::HardwareConfig &hw,
                                       spark2::MotorDriverKind driver)
{
    return (driver == spark2::MotorDriverKind::Tl) ? hw.canfd.channel : hw.can.channel;
}

int Spark2::init_can_bus(const spark2::HardwareConfig &hw, int *chn_idx_out, int *chn_num_out)
{
    const bool has_tl = [&hw]() {
        for (const auto &slot : hw.motors) {
            if (slot.enabled && slot.driver == spark2::MotorDriverKind::Tl) {
                return true;
            }
        }
        return false;
    }();
    const bool has_rmd = [&hw]() {
        for (const auto &slot : hw.motors) {
            if (slot.enabled && slot.driver == spark2::MotorDriverKind::Rmd) {
                return true;
            }
        }
        return false;
    }();

    int chn_idx[2] = {hw.canfd.channel, hw.can.channel};
    int node_num[2] = {1, 1};
    int chn_num = (hw.canfd.channel == hw.can.channel) ? 1 : 2;
    if (chn_num == 1) {
        chn_idx[0] = has_tl ? hw.canfd.channel : hw.can.channel;
    }

    if (!has_tl && has_rmd) {
        chn_idx[0] = hw.can.channel;
        chn_num = 1;
    } else if (has_tl && !has_rmd) {
        chn_idx[0] = hw.canfd.channel;
        chn_num = 1;
    }

    *chn_num_out = chn_num;
    for (int i = 0; i < chn_num; ++i) {
        chn_idx_out[i] = chn_idx[i];
        node_num[i] = 1;
    }
    (void)hw.can.baudrate;
    return cxt_can_init(&bus_, VCI_USBCAN2, 0, chn_num, chn_idx, node_num);
}

std::vector<std::unique_ptr<JointActuatorBase>>
Spark2::build_joints(const spark2::HardwareConfig &hw, const int *chn_idx, int chn_num)
{
    bool need_tl = false;
    bool need_rmd = false;
    for (const auto &slot : hw.motors) {
        if (!slot.enabled) {
            continue;
        }
        if (slot.driver == spark2::MotorDriverKind::Tl) {
            need_tl = true;
        } else {
            need_rmd = true;
        }
    }

    int tl_phys = 0;
    int rmd_phys = 0;
    if (need_tl) {
        tl_phys = phys_channel_for_driver(hw, spark2::MotorDriverKind::Tl);
        if (logical_channel_index(chn_idx, chn_num, tl_phys) < 0) {
            throw std::runtime_error("Spark2: CAN channel not opened for TL motors");
        }
    }
    if (need_rmd) {
        rmd_phys = phys_channel_for_driver(hw, spark2::MotorDriverKind::Rmd);
        if (logical_channel_index(chn_idx, chn_num, rmd_phys) < 0) {
            throw std::runtime_error("Spark2: CAN channel not opened for RMD motors");
        }
    }

    std::vector<std::unique_ptr<JointActuatorBase>> joints(static_cast<std::size_t>(hw.num_joints));
    for (int i = 0; i < hw.num_joints; ++i) {
        const auto &slot = hw.motors[static_cast<std::size_t>(i)];
        if (!slot.enabled) {
            continue;
        }
        const float gear = static_cast<float>(slot.gear_ratio);
        if (slot.driver == spark2::MotorDriverKind::Tl) {
            const int tl_log = logical_channel_index(chn_idx, chn_num, tl_phys);
            joints[static_cast<std::size_t>(i)] = std::make_unique<TlCanopenActuator>(
                &bus_, tl_log, static_cast<uint8_t>(slot.id), gear, slot.resolution);
        } else {
            const int rmd_log = logical_channel_index(chn_idx, chn_num, rmd_phys);
            joints[static_cast<std::size_t>(i)] = std::make_unique<RmdCanActuator>(
                &bus_, rmd_log, static_cast<uint32_t>(slot.id), RMD_V4, gear, 8192,
                slot.has_brake);
        }
    }
    return joints;
}

void Spark2::load_hardware_config()
{
    std::string err;
    if (!spark2::load_hardware_yaml(config_path_, &hardware_, &err)) {
        throw std::runtime_error("Spark2: failed to load hardware YAML '" + config_path_ + "': " + err);
    }
}

void Spark2::initialize_from_hardware_config()
{
    if (hardware_.num_joints != kJointSize_) {
        std::ostringstream os;
        os << "Spark2: YAML num_joints (" << hardware_.num_joints << ") != joint count (" << kJointSize_
           << "), config: '" << config_path_ << "'";
        throw std::runtime_error(os.str());
    }
    if (hardware_.motors.size() != static_cast<std::size_t>(kJointSize_)) {
        std::ostringstream os;
        os << "Spark2: hardware.motor axis count (" << hardware_.motors.size() << ") != joint count ("
           << kJointSize_ << "), config: '" << config_path_ << "'";
        throw std::runtime_error(os.str());
    }
    for (int i = 0; i < kJointSize_; ++i) {
        const auto &slot = hardware_.motors[static_cast<std::size_t>(i)];
        existing_motors_[static_cast<std::size_t>(i)] = slot.enabled;
        enable_motors_[static_cast<std::size_t>(i)] = slot.enabled;
        if (slot.enabled) {
            if (!joints_[static_cast<std::size_t>(i)]) {
                std::ostringstream os;
                os << "Spark2: hardware.motor.enabled[" << i << "] is true but joint " << i
                   << " is null, config: '" << config_path_ << "'";
                throw std::runtime_error(os.str());
            }
            const bool want_tl = (slot.driver == spark2::MotorDriverKind::Tl);
            const bool is_tl = (joints_[static_cast<std::size_t>(i)]->joint_kind() == JointKind::TlCanopen);
            if (want_tl != is_tl) {
                std::ostringstream os;
                os << "Spark2: hardware.motor.motor_type[" << i << "] does not match injected joint kind "
                   << "(config '" << config_path_ << "')";
                throw std::runtime_error(os.str());
            }
        }
    }
    hardware_enabled_.assign(static_cast<std::size_t>(kJointSize_), false);
    joint_modes_.assign(static_cast<std::size_t>(kJointSize_), ControlType::kInvalid);
}

Spark2::Spark2(const std::string &name, const std::string &config_path)
    : ArmBase(name, load_joint_count_or_throw(config_path)), config_path_(resolve_config_path(config_path)),
      owns_bus_(true)
{
    load_hardware_config();

    int chn_idx[2] = {0, 0};
    int chn_num = 0;
    if (init_can_bus(hardware_, chn_idx, &chn_num) != 0) {
        throw std::runtime_error("Spark2: cxt_can_init failed");
    }
    bus_opened_ = true;
    joints_ = build_joints(hardware_, chn_idx, chn_num);
    initialize_from_hardware_config();
}

Spark2::Spark2(const std::string &name, std::vector<std::unique_ptr<JointActuatorBase>> joints,
                   const std::string &config_path)
    : ArmBase(name, static_cast<int>(joints.size())), config_path_(config_path), joints_(std::move(joints))
{
    load_hardware_config();
    initialize_from_hardware_config();
}

Spark2::~Spark2()
{
    if (owns_bus_ && bus_opened_) {
        close();
    }
}

bool Spark2::is_tl_joint(int joint_i) const
{
    return isValidIdx(joint_i) && joints_[static_cast<std::size_t>(joint_i)] &&
           joints_[static_cast<std::size_t>(joint_i)]->joint_kind() == JointKind::TlCanopen;
}

RetCode Spark2::hardware_enable_joint(int joint_i)
{
    
    if (!isValidIdx(joint_i)) {
        return RetCode::kInvalidIndex;
    }
    JointActuatorBase *jp = joints_[static_cast<std::size_t>(joint_i)].get();
    if (!jp) {
        return RetCode::kFailed;
    }
    if (hardware_enabled_[static_cast<std::size_t>(joint_i)]) {
        
        return RetCode::kSuccess;
    }
    if (jp->enable() != 0) {
        
        return RetCode::kFailed;
    }
    hardware_enabled_[static_cast<std::size_t>(joint_i)] = true;
    enable_motors_[static_cast<std::size_t>(joint_i)] = true;

    // For TL joints: read current position and set it as initial target to hold position
    if (is_tl_joint(joint_i)) {
        if (jp->read_status() == 0) {
            double cur_pos = jp->get_position();
            jp->set_absolute_position_rad(cur_pos);
            
        }
    }

    
    return RetCode::kSuccess;
}

RetCode Spark2::hardware_disable_joint(int joint_i)
{
    
    if (!isValidIdx(joint_i)) {
        return RetCode::kInvalidIndex;
    }
    JointActuatorBase *jp = joints_[static_cast<std::size_t>(joint_i)].get();
    if (!jp) {
        return RetCode::kFailed;
    }
    if (!hardware_enabled_[static_cast<std::size_t>(joint_i)]) {
        return RetCode::kSuccess;
    }
    const int r = jp->disable();
    hardware_enabled_[static_cast<std::size_t>(joint_i)] = false;
    enable_motors_[static_cast<std::size_t>(joint_i)] = false;
    return r == 0 ? RetCode::kSuccess : RetCode::kFailed;
}

double Spark2::user_position_to_actuator_rad(int joint_i, float user_rad) const
{
    const auto &slot = hardware_.motors[static_cast<std::size_t>(joint_i)];
    const double s = static_cast<double>(slot.sign);
    return static_cast<double>(user_rad) * s + slot.motor_offset_rad;
}

double Spark2::actuator_position_to_user_rad(int joint_i, double actuator_rad) const
{
    const auto &slot = hardware_.motors[static_cast<std::size_t>(joint_i)];
    const double s = static_cast<double>(slot.sign);
    return (actuator_rad - slot.motor_offset_rad) * s;
}

double Spark2::user_velocity_to_actuator(int joint_i, float user_vel) const
{
    const double s = static_cast<double>(hardware_.motors[static_cast<std::size_t>(joint_i)].sign);
    return static_cast<double>(user_vel) * s;
}

double Spark2::actuator_velocity_to_user(int joint_i, double actuator_vel) const
{
    const double s = static_cast<double>(hardware_.motors[static_cast<std::size_t>(joint_i)].sign);
    return actuator_vel * s;
}

double Spark2::user_current_to_actuator(int joint_i, float user_cur) const
{
    const double s = static_cast<double>(hardware_.motors[static_cast<std::size_t>(joint_i)].sign);
    return static_cast<double>(user_cur) * s;
}

double Spark2::actuator_current_to_user(int joint_i, double actuator_cur) const
{
    const double s = static_cast<double>(hardware_.motors[static_cast<std::size_t>(joint_i)].sign);
    return actuator_cur * s;
}

RetCode Spark2::joint_rc(int r)
{
    return r == 0 ? RetCode::kSuccess : RetCode::kFailed;
}

JointControlMode Spark2::control_type_to_joint_mode(ControlType mode)
{
    switch (mode) {
    case ControlType::kPosition:
        return JointControlMode::Position;
    case ControlType::kVelocity:
        return JointControlMode::Velocity;
    case ControlType::kTorque:
        return JointControlMode::Current;
    default:
        return JointControlMode::OpenLoop;
    }
}

RetCode Spark2::open()
{
    bool ok = true;
    const JointControlMode jm = JointControlMode::Position;

    // Set operation mode BEFORE enable to match test_spark2_tl_motor behavior
    for (int i = 0; i < kJointSize_; ++i) {
        if (!existing_motors_[static_cast<std::size_t>(i)] || !joints_[static_cast<std::size_t>(i)]) {
            continue;
        }
        joints_[static_cast<std::size_t>(i)]->select_mode(jm);
    }

    for (int i = 0; i < kJointSize_; ++i) {
        if (!existing_motors_[static_cast<std::size_t>(i)] || !joints_[static_cast<std::size_t>(i)]) {
            continue;
        }
        JointActuatorBase &j = *joints_[static_cast<std::size_t>(i)];
        ok &= (j.clear_error() == 0);
        ok &= (hardware_enable_joint(i) == RetCode::kSuccess);
        if (j.read_status() != 0) {
            ok = false;
        }
    }
    current_mode_ = ControlType::kPosition;
    std::fill(joint_modes_.begin(), joint_modes_.end(), ControlType::kPosition);
    return ok ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode Spark2::close()
{
    bool ok = true;
    for (int i = 0; i < kJointSize_; ++i) {
        if (!existing_motors_[static_cast<std::size_t>(i)]) {
            continue;
        }
        ok &= (hardware_disable_joint(i) == RetCode::kSuccess);
        enable_motors_[static_cast<std::size_t>(i)] = false;
    }
    if (owns_bus_ && bus_opened_) {
        ok &= (cxt_can_close(&bus_) == 0);
        bus_opened_ = false;
    }
    return ok ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode Spark2::setMode(ControlType mode)
{
    if (mode == ControlType::kInvalid) {
        current_mode_ = ControlType::kInvalid;
        return RetCode::kSuccess;
    }
    if (current_mode_ == mode) {
        
        return RetCode::kSuccess;  // Already in the desired mode
    }
    bool ok = true;

    // TL (CiA-402) motors handle mode switching inside select_mode() via
    // tl_switch_mode_safely(), which manages the full state machine.
    // RMD motors determine mode by command type (0xA1/0xA2/0xA4) and have
    // no CiA-402 state machine — disable/re-enable only causes brake
    // release gap and joint drop. Skip for both.
    const bool needs_reenable = false;

    for (int i = 0; i < kJointSize_; ++i) {
        if (!existing_motors_[static_cast<std::size_t>(i)] || !joints_[static_cast<std::size_t>(i)]) {
            continue;
        }
        const bool reenable_joint = needs_reenable && !is_tl_joint(i);
        if (reenable_joint) {
            if (hardware_enabled_[static_cast<std::size_t>(i)]) {
                joints_[static_cast<std::size_t>(i)]->disable();
                hardware_enabled_[static_cast<std::size_t>(i)] = false;
            }
            int en_ret = joints_[static_cast<std::size_t>(i)]->enable();
            ok &= (en_ret == 0);
            if (en_ret == 0) {
                hardware_enabled_[static_cast<std::size_t>(i)] = true;
            }
        }

        ControlType joint_mode = mode;
        const JointControlMode jm = control_type_to_joint_mode(joint_mode);
        int sel_ret = joints_[static_cast<std::size_t>(i)]->select_mode(jm);
        ok &= (sel_ret == 0);
        joint_modes_[static_cast<std::size_t>(i)] = joint_mode;
    }
    if (!ok) {
        return RetCode::kFailed;
    }
    current_mode_ = mode;
    
    printf("Spark2::setMode: switched to mode %d, waiting for stabilization...\n", (int)mode);
    sleep_period(1000000);
    printf("Spark2::setMode: switched to mode %d\n", (int)mode);
    return RetCode::kSuccess;
}

RetCode Spark2::setCommand(int joint_i, float val)
{
    double actuator_val = user_position_to_actuator_rad(joint_i, val);
    
    if (!isValidIdx(joint_i) || !enable_motors_[static_cast<std::size_t>(joint_i)]) {
        
        return RetCode::kInvalidIndex;
    }
    if (current_mode_ == ControlType::kInvalid) {
        
        return RetCode::kFailed;
    }
    JointActuatorBase &j = *joints_[static_cast<std::size_t>(joint_i)];
    int r = 0;
    ControlType joint_mode = joint_modes_[static_cast<std::size_t>(joint_i)];
    switch (joint_mode) {
    case ControlType::kPosition:
        r = j.set_absolute_position_rad(actuator_val);
        
        break;
    case ControlType::kVelocity:
        r = j.set_velocity(user_velocity_to_actuator(joint_i, val));
        break;
    case ControlType::kTorque:
        r = j.set_current(user_current_to_actuator(joint_i, val));
        break;
    default:
        return RetCode::kFailed;
    }
    return joint_rc(r);
}

RetCode Spark2::getPosition(int joint_i, float *val)
{
    if (!val) {
        return RetCode::kInvalidInput;
    }
    if (!isValidIdx(joint_i) || !enable_motors_[static_cast<std::size_t>(joint_i)]) {
        return RetCode::kInvalidIndex;
    }
    JointActuatorBase *jp = joints_[static_cast<std::size_t>(joint_i)].get();
    if (!jp) {
        return RetCode::kFailed;
    }
    if (!read_joint_position(jp)) {
        *val = 0.0f;  // 安全默认值
        return RetCode::kFailed;
    }
    double actuator_pos = jp->get_position();
    const auto &slot = hardware_.motors[static_cast<std::size_t>(joint_i)];
    const double s = static_cast<double>(slot.sign);
    double user_pos = (actuator_pos - slot.motor_offset_rad) * s;
    *val = static_cast<float>(user_pos);
    
    return RetCode::kSuccess;
}

RetCode Spark2::getVelocity(int joint_i, float *val)
{
    if (!val) {
        return RetCode::kInvalidInput;
    }
    if (!isValidIdx(joint_i) || !enable_motors_[static_cast<std::size_t>(joint_i)]) {
        return RetCode::kInvalidIndex;
    }
    JointActuatorBase *jp = joints_[static_cast<std::size_t>(joint_i)].get();
    if (!jp) {
        return RetCode::kFailed;
    }
    if (dynamic_cast<RmdCanActuator *>(jp) == nullptr && jp->read_status() != 0) {
        *val = 0.0f;  // 安全默认值
        return RetCode::kFailed;
    }
    *val = static_cast<float>(actuator_velocity_to_user(joint_i, jp->get_velocity()));
    return RetCode::kSuccess;
}

RetCode Spark2::getTorque(int joint_i, float *val)
{
    if (!val) {
        return RetCode::kInvalidInput;
    }
    if (!isValidIdx(joint_i) || !enable_motors_[static_cast<std::size_t>(joint_i)]) {
        return RetCode::kInvalidIndex;
    }
    JointActuatorBase *jp = joints_[static_cast<std::size_t>(joint_i)].get();
    if (!jp) {
        return RetCode::kFailed;
    }
    if (dynamic_cast<RmdCanActuator *>(jp) == nullptr && jp->read_status() != 0) {
        *val = 0.0f;  // 安全默认值
        return RetCode::kFailed;
    }
    double raw_tor = jp->get_current();
    double converted_tor = actuator_current_to_user(joint_i, raw_tor);
    *val = static_cast<float>(converted_tor);
    return RetCode::kSuccess;
}

RetCode Spark2::getState(int joint_i, float *pos, float *vel, float *tor, int read_delay_us)
{
    if (!isValidIdx(joint_i) || !enable_motors_[static_cast<std::size_t>(joint_i)]) {
        return RetCode::kInvalidIndex;
    }
    JointActuatorBase *jp = joints_[static_cast<std::size_t>(joint_i)].get();
    if (!jp) {
        return RetCode::kFailed;
    }
    if (pos || vel || tor) {
        if (!refresh_joint_feedback(jp, pos != nullptr, vel != nullptr || tor != nullptr)) {
            return RetCode::kFailed;
        }
    }
    if (pos) {
        *pos = static_cast<float>(actuator_position_to_user_rad(joint_i, jp->get_position()));
    }
    if (vel) {
        *vel = static_cast<float>(actuator_velocity_to_user(joint_i, jp->get_velocity()));
    }
    if (tor) {
        double raw_tor = jp->get_current();
        double converted_tor = actuator_current_to_user(joint_i, raw_tor);
        *tor = static_cast<float>(converted_tor);
    }
    if (read_delay_us > 0) {
        sleep_period(read_delay_us);
    }
    return RetCode::kSuccess;
}

RetCode Spark2::getState(std::vector<float> *pos, std::vector<float> *vel, std::vector<float> *tor,
                            int read_delay_us)
{
    return ArmBase::getState(pos, vel, tor, read_delay_us);
}

RetCode Spark2::setCommandAndGetState(const std::vector<float> &set_val, std::vector<float> *pos,
                                         std::vector<float> *vel, std::vector<float> *tor,
                                         int set_delay_us, int read_delay_us)
{
    if (!isValidInput(set_val)) {
        return RetCode::kInvalidInput;
    }
    const bool need_read_pos = (pos != nullptr && isValidInput(*pos));
    const bool need_read_vel = (vel != nullptr && isValidInput(*vel));
    const bool need_read_tor = (tor != nullptr && isValidInput(*tor));
    if ((pos && !need_read_pos) || (vel && !need_read_vel) || (tor && !need_read_tor)) {
        return RetCode::kInvalidInput;
    }

    bool success = true;
    std::vector<int> rmd_channels;
    std::vector<int> rmd_indices;
    std::vector<int> tl_channels;
    std::vector<int> tl_indices;

    for (int joint_i = 0; joint_i < kJointSize_; ++joint_i) {
        if (!isValidIdx(joint_i) || !enable_motors_[static_cast<std::size_t>(joint_i)]) {
            continue;
        }
        JointActuatorBase *joint = joints_[static_cast<std::size_t>(joint_i)].get();

        /* 按驱动器类型分流：TL → 批量通道收集，RMD → 批量通道收集 */
        TlCanopenActuator *tl = dynamic_cast<TlCanopenActuator *>(joint);
        if (tl) {
            tl_indices.push_back(joint_i);
            if (std::find(tl_channels.begin(), tl_channels.end(), tl->can_channel()) == tl_channels.end()) {
                tl_channels.push_back(tl->can_channel());
            }
            continue;
        }

        RmdCanActuator *rmd = dynamic_cast<RmdCanActuator *>(joint);
        if (!rmd) {
            continue;
        }

        rmd_indices.push_back(joint_i);
        if (std::find(rmd_channels.begin(), rmd_channels.end(), rmd->can_channel()) == rmd_channels.end()) {
            rmd_channels.push_back(rmd->can_channel());
        }
    }

    for (int channel : rmd_channels) {
        std::vector<int> group_indices;
        std::vector<RmdCanActuator *> group_actuators;
        std::vector<rmd_motor> group_motors;

        for (int joint_i : rmd_indices) {
            RmdCanActuator *rmd = dynamic_cast<RmdCanActuator *>(joints_[static_cast<std::size_t>(joint_i)].get());
            if (!rmd || rmd->can_channel() != channel) {
                continue;
            }
            const ControlType joint_mode = joint_modes_[static_cast<std::size_t>(joint_i)];
            const JointControlMode jm = control_type_to_joint_mode(joint_mode);
            double actuator_value = 0.0;
            switch (joint_mode) {
            case ControlType::kPosition:
                actuator_value = user_position_to_actuator_rad(joint_i, set_val[static_cast<std::size_t>(joint_i)]);
                break;
            case ControlType::kVelocity:
                actuator_value = user_velocity_to_actuator(joint_i, set_val[static_cast<std::size_t>(joint_i)]);
                break;
            case ControlType::kTorque:
                actuator_value = user_current_to_actuator(joint_i, set_val[static_cast<std::size_t>(joint_i)]);
                break;
            default:
                success = false;
                continue;
            }

            rmd_motor prepared{};
            if (rmd->prepare_command_motor(jm, actuator_value, &prepared) != 0) {
                success = false;
                continue;
            }
            group_indices.push_back(joint_i);
            group_actuators.push_back(rmd);
            group_motors.push_back(prepared);
        }

        if (group_motors.empty()) {
            continue;
        }

        const int old_node_num = bus_.node_num[channel];
        bus_.node_num[channel] = static_cast<int>(group_motors.size());
        const int ret = cxt_can_rmd_motor_exchange(&bus_, channel, group_motors.data());
        bus_.node_num[channel] = old_node_num;
        if (ret != 0) {
            success = false;
            continue;
        }
        if (set_delay_us > 0) {
            sleep_period(set_delay_us);
        }

        for (std::size_t i = 0; i < group_actuators.size(); ++i) {
            RmdCanActuator *rmd = group_actuators[i];
            const int joint_i = group_indices[i];
            rmd->apply_command_feedback(group_motors[i]);
            if (need_read_pos) {
                success &= (rmd->read_absolute_position() == 0);
            }
            if (need_read_pos) {
                (*pos)[static_cast<std::size_t>(joint_i)] =
                    static_cast<float>(actuator_position_to_user_rad(joint_i, rmd->get_position()));
            }
            if (need_read_vel) {
                (*vel)[static_cast<std::size_t>(joint_i)] =
                    static_cast<float>(actuator_velocity_to_user(joint_i, rmd->get_velocity()));
            }
            if (need_read_tor) {
                (*tor)[static_cast<std::size_t>(joint_i)] =
                    static_cast<float>(actuator_current_to_user(joint_i, rmd->get_current()));
            }
            if (read_delay_us > 0) {
                sleep_period(read_delay_us);
            }
        }
    }

    /* ---- TL 批量 SDO exchange：按 CAN 通道分组，SET+READ 各 1 次 exchange ---- */
    for (int channel : tl_channels) {
        std::vector<int>                  group_indices;
        std::vector<TlCanopenActuator *> group_actuators;
        std::vector<tl_motor>            group_motors;

        for (int joint_i : tl_indices) {
            TlCanopenActuator *tl = dynamic_cast<TlCanopenActuator *>(joints_[static_cast<std::size_t>(joint_i)].get());
            if (!tl || tl->can_channel() != channel) {
                continue;
            }
            const ControlType joint_mode = joint_modes_[static_cast<std::size_t>(joint_i)];
            const JointControlMode jm = control_type_to_joint_mode(joint_mode);
            double actuator_value = 0.0;
            switch (joint_mode) {
            case ControlType::kPosition:
                actuator_value = user_position_to_actuator_rad(joint_i, set_val[static_cast<std::size_t>(joint_i)]);
                break;
            case ControlType::kVelocity:
                actuator_value = user_velocity_to_actuator(joint_i, set_val[static_cast<std::size_t>(joint_i)]);
                break;
            case ControlType::kTorque:
                actuator_value = user_current_to_actuator(joint_i, set_val[static_cast<std::size_t>(joint_i)]);
                break;
            default:
                success = false;
                continue;
            }

            tl_motor prepared{};
            tl->prepare_command_motor(jm, actuator_value, &prepared);
            group_indices.push_back(joint_i);
            group_actuators.push_back(tl);
            group_motors.push_back(prepared);
        }

        if (group_motors.empty()) {
            continue;
        }

        const int old_node_num = bus_.node_num[channel];
        const int n_motors = static_cast<int>(group_motors.size());

        /* Phase 1: 批量 SET — 所有 TL 电机目标位置/速度/力矩一次性发出 */
        bus_.node_num[channel] = n_motors;
        int ret = cxt_can_tl_motor_exchange(&bus_, channel, group_motors.data());
        bus_.node_num[channel] = old_node_num;
        if (ret != 0) {
            success = false;
            continue;
        }
        if (set_delay_us > 0) {
            sleep_period(set_delay_us);
        }

        /* Phase 2: 批量 READ position */
        for (int i = 0; i < n_motors; ++i) {
            tl_motor_get_position(&group_motors[i]);
        }
        bus_.node_num[channel] = n_motors;
        ret = cxt_can_tl_motor_exchange(&bus_, channel, group_motors.data());
        bus_.node_num[channel] = old_node_num;
        if (ret != 0) {
            success = false;
            continue;
        }

        /* Phase 3: 批量 READ velocity */
        for (int i = 0; i < n_motors; ++i) {
            tl_motor_get_velocity(&group_motors[i]);
        }
        bus_.node_num[channel] = n_motors;
        ret = cxt_can_tl_motor_exchange(&bus_, channel, group_motors.data());
        bus_.node_num[channel] = old_node_num;
        if (ret != 0) {
            success = false;
            continue;
        }

        /* Phase 4: 批量 READ torque */
        for (int i = 0; i < n_motors; ++i) {
            tl_motor_get_torque(&group_motors[i]);
        }
        bus_.node_num[channel] = n_motors;
        ret = cxt_can_tl_motor_exchange(&bus_, channel, group_motors.data());
        bus_.node_num[channel] = old_node_num;
        if (ret != 0) {
            success = false;
            continue;
        }

        /* 将所有 exchange 中累积的反馈一次性写回各 actuator */
        for (std::size_t i = 0; i < group_actuators.size(); ++i) {
            TlCanopenActuator *tl = group_actuators[i];
            const int joint_i = group_indices[i];
            tl->apply_feedback(group_motors[i]);
            if (need_read_pos) {
                (*pos)[static_cast<std::size_t>(joint_i)] =
                    static_cast<float>(actuator_position_to_user_rad(joint_i, tl->get_position()));
            }
            if (need_read_vel) {
                (*vel)[static_cast<std::size_t>(joint_i)] =
                    static_cast<float>(actuator_velocity_to_user(joint_i, tl->get_velocity()));
            }
            if (need_read_tor) {
                (*tor)[static_cast<std::size_t>(joint_i)] =
                    static_cast<float>(actuator_current_to_user(joint_i, tl->get_current()));
            }
            if (read_delay_us > 0) {
                sleep_period(read_delay_us);
            }
        }
    }

    return success ? RetCode::kSuccess : RetCode::kFailed;
}

RetCode Spark2::updateEncoderZeroOffset(int joint_i, float offset)
{
    if (!isValidIdx(joint_i)) {
        return RetCode::kInvalidIndex;
    }
    hardware_.motors[static_cast<std::size_t>(joint_i)].motor_offset_rad = static_cast<double>(offset);
    return RetCode::kSuccess;
}

RetCode Spark2::enableMotor(int joint_i, bool enable)
{
    if (joint_i < 0 || joint_i >= kJointSize_) {
        return RetCode::kInvalidIndex;
    }
    if (!existing_motors_[static_cast<std::size_t>(joint_i)]) {
        return RetCode::kInvalidIndex;
    }
    RetCode rc;
    if (enable) {
        rc = hardware_enable_joint(joint_i);
    } else {
        rc = hardware_disable_joint(joint_i);
    }
    enable_motors_[static_cast<std::size_t>(joint_i)] = enable;
    return rc;
}

RetCode Spark2::enableMotors(const std::vector<bool> &enable)
{
    if (!isValidInput(enable)) {
        return RetCode::kInvalidInput;
    }
    bool ok = true;
    for (int joint_i = 0; joint_i < kJointSize_; ++joint_i) {
        if (!existing_motors_[static_cast<std::size_t>(joint_i)]) {
            continue;
        }
        if (enable[static_cast<std::size_t>(joint_i)]) {
            if (!is_tl_joint(joint_i) || !hardware_enabled_[static_cast<std::size_t>(joint_i)]) {
                ok &= (hardware_enable_joint(joint_i) == RetCode::kSuccess);
            }
        } else {
            ok &= (hardware_disable_joint(joint_i) == RetCode::kSuccess);
        }
        enable_motors_[static_cast<std::size_t>(joint_i)] = enable[static_cast<std::size_t>(joint_i)];
    }
    return ok ? RetCode::kSuccess : RetCode::kFailed;
}
